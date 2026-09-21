#include <chrono>
#include <cstddef>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace brain_robot_pick_place
{

class ActiveScanSupervisor : public rclcpp::Node
{
public:
  explicit ActiveScanSupervisor(const rclcpp::NodeOptions & options)
  : Node("active_scan_supervisor", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    point_cloud_topic_ = declare_parameter<std::string>(
      "point_cloud_topic", "/camera/depth/points");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/piper_moveit_joint_states");
    planning_scene_topic_ = declare_parameter<std::string>(
      "planning_scene_topic", "/monitored_planning_scene");
    reference_frame_ = declare_parameter<std::string>("reference_frame", "base_link");
    camera_optical_frame_ = declare_parameter<std::string>(
      "camera_optical_frame", "camera_color_optical_frame");
    map_timeout_s_ = declare_parameter<double>("map_timeout_s", 1.0);
    joint_state_timeout_s_ = declare_parameter<double>("joint_state_timeout_s", 0.3);
    initial_observe_stable_frames_ = declare_parameter<int>("initial_observe_stable_frames", 3);

    if (map_timeout_s_ <= 0.0 || joint_state_timeout_s_ <= 0.0 ||
      initial_observe_stable_frames_ <= 0)
    {
      throw std::runtime_error("Active scan freshness parameters must be positive.");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    point_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        last_point_cloud_time_ = std::chrono::steady_clock::now();
        ++stable_point_cloud_frames_;
        latest_point_count_ = static_cast<std::size_t>(message->width) * message->height;
      });
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, sensor_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr) {
        last_joint_state_time_ = std::chrono::steady_clock::now();
      });
    planning_scene_subscription_ = create_subscription<moveit_msgs::msg::PlanningScene>(
      planning_scene_topic_, rclcpp::QoS(1).reliable(),
      [this](moveit_msgs::msg::PlanningScene::ConstSharedPtr message) {
        last_planning_scene_time_ = std::chrono::steady_clock::now();
        planning_scene_seen_ = true;
        planning_scene_has_octomap_ = !message->world.octomap.octomap.id.empty();
      });

    const auto diagnostic_qos = rclcpp::QoS(1).reliable().transient_local();
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/brain_robot_active_scan/state", diagnostic_qos);
    diagnostic_publisher_ = create_publisher<std_msgs::msg::String>(
      "/brain_robot_active_scan/diagnostic", diagnostic_qos);
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {PublishStatus();});
  }

private:
  static double AgeSeconds(const std::chrono::steady_clock::time_point & received_time)
  {
    if (received_time == std::chrono::steady_clock::time_point{}) {
      return -1.0;
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - received_time).count();
  }

  bool CameraTransformAvailable(std::string & detail)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        reference_frame_, camera_optical_frame_, tf2::TimePointZero);
      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(3);
      stream << "base<-camera=(" << transform.transform.translation.x << ',' <<
        transform.transform.translation.y << ',' << transform.transform.translation.z << ')';
      detail = stream.str();
      return true;
    } catch (const tf2::TransformException & exception) {
      detail = exception.what();
      return false;
    }
  }

  void PublishStatus()
  {
    const double point_cloud_age_s = AgeSeconds(last_point_cloud_time_);
    const double joint_state_age_s = AgeSeconds(last_joint_state_time_);
    const double planning_scene_age_s = AgeSeconds(last_planning_scene_time_);
    std::string tf_detail;
    const bool camera_tf_available = CameraTransformAvailable(tf_detail);

    std::string state;
    if (point_cloud_age_s < 0.0 || point_cloud_age_s > map_timeout_s_) {
      stable_point_cloud_frames_ = 0;
      state = "WAITING_FOR_POINTCLOUD";
    } else if (joint_state_age_s < 0.0 || joint_state_age_s > joint_state_timeout_s_) {
      state = "WAITING_FOR_JOINT_FEEDBACK";
    } else if (!camera_tf_available) {
      state = "WAITING_FOR_CAMERA_TF";
    } else if (!planning_scene_seen_) {
      state = "WAITING_FOR_PLANNING_SCENE";
    } else if (stable_point_cloud_frames_ < initial_observe_stable_frames_) {
      state = "INITIAL_STILL_OBSERVE";
    } else {
      state = "MAP_INPUT_READY";
    }

    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);

    std::ostringstream diagnostic;
    diagnostic.setf(std::ios::fixed);
    diagnostic.precision(3);
    diagnostic << "state=" << state <<
      " point_cloud_age_s=" << point_cloud_age_s <<
      " point_count=" << latest_point_count_ <<
      " stable_frames=" << stable_point_cloud_frames_ <<
      " joint_state_age_s=" << joint_state_age_s <<
      " planning_scene_age_s=" << planning_scene_age_s <<
      " planning_scene_octomap_payload=" << (planning_scene_has_octomap_ ? "true" : "false") <<
      " tf=" << tf_detail;
    message.data = diagnostic.str();
    diagnostic_publisher_->publish(message);
  }

  std::string point_cloud_topic_;
  std::string joint_state_topic_;
  std::string planning_scene_topic_;
  std::string reference_frame_;
  std::string camera_optical_frame_;
  double map_timeout_s_{1.0};
  double joint_state_timeout_s_{0.3};
  int initial_observe_stable_frames_{3};
  std::size_t latest_point_count_{0};
  int stable_point_cloud_frames_{0};
  bool planning_scene_seen_{false};
  bool planning_scene_has_octomap_{false};
  std::chrono::steady_clock::time_point last_point_cloud_time_{};
  std::chrono::steady_clock::time_point last_joint_state_time_{};
  std::chrono::steady_clock::time_point last_planning_scene_time_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace brain_robot_pick_place

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<brain_robot_pick_place::ActiveScanSupervisor>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
