#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace brain_robot_pick_place
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

enum class ControlState
{
  IDLE,
  PREPARE,
  SEARCH,
  LOCAL_SEARCH,
  ALIGN,
  LEVEL_ALIGN,
  LEVEL_RECOVERY,
  FINAL_ALIGN,
  GRASP_READY,
  STOPPED,
  FAULT,
};

enum class FullSearchPhase
{
  DESCEND,
  BOTTOM_SCAN,
  ASCEND,
  TOP_SCAN,
};

const char * StateName(ControlState state)
{
  switch (state) {
    case ControlState::IDLE: return "IDLE";
    case ControlState::PREPARE: return "PREPARE";
    case ControlState::SEARCH: return "SEARCH";
    case ControlState::LOCAL_SEARCH: return "LOCAL_SEARCH";
    case ControlState::ALIGN: return "ALIGN";
    case ControlState::LEVEL_ALIGN: return "LEVEL_ALIGN";
    case ControlState::LEVEL_RECOVERY: return "LEVEL_RECOVERY";
    case ControlState::FINAL_ALIGN: return "FINAL_ALIGN";
    case ControlState::GRASP_READY: return "GRASP_READY";
    case ControlState::STOPPED: return "STOPPED";
    case ControlState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char * SearchPhaseName(FullSearchPhase phase)
{
  switch (phase) {
    case FullSearchPhase::DESCEND: return "SEARCH_DOWN";
    case FullSearchPhase::BOTTOM_SCAN: return "BOTTOM_SCAN";
    case FullSearchPhase::ASCEND: return "SEARCH_UP";
    case FullSearchPhase::TOP_SCAN: return "TOP_SCAN";
  }
  return "UNKNOWN";
}

class VisualSearchController : public rclcpp::Node
{
public:
  explicit VisualSearchController(const rclcpp::NodeOptions & options)
  : Node("visual_search_controller", options)
  {
    LoadParameters();
    CreateRosInterfaces();
    ValidateParameters();
    PublishStateLocked("WAITING_FOR_START");

    RCLCPP_INFO(
      get_logger(),
      "Visual controller ready: backend=%s, auto motion disabled until ~/start is called.",
      backend_.c_str());
  }

  ~VisualSearchController() override
  {
    cancel_requested_ = true;
    if (move_group_) {
      move_group_->stop();
    }
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    if (prepare_thread_.joinable()) {
      prepare_thread_.join();
    }
  }

  void SetMoveGroup(const std::shared_ptr<MoveGroupInterface> & move_group)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    move_group_ = move_group;
  }

private:
  template<typename T>
  T ParameterOr(const std::string & name, const T & fallback)
  {
    T value = fallback;
    get_parameter_or(name, value, fallback);
    return value;
  }

  void LoadParameters()
  {
    backend_ = ParameterOr<std::string>("backend", "simulation");
    real_motion_enabled_ = ParameterOr<bool>("real_motion_enabled", false);
    observe_pose_calibrated_ = ParameterOr<bool>("observe_pose_calibrated", true);
    execute_prepare_ = ParameterOr<bool>("execute_prepare", true);
    skip_prepare_ = ParameterOr<bool>("skip_prepare", false);

    joint_names_ = ParameterOr<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    observe_joint_positions_ = ParameterOr<std::vector<double>>(
      "observe_joint_positions", {0.0, 0.98, -0.75, 0.0, 0.30, 0.0});
    horizontal_joint_ = ParameterOr<std::string>("horizontal_joint", "joint1");
    vertical_joint_ = ParameterOr<std::string>("vertical_joint", "joint5");
    horizon_joint_names_ = ParameterOr<std::vector<std::string>>(
      "horizon_joint_names", {"joint4", "joint6"});
    horizon_lock_positions_ = ParameterOr<std::vector<double>>(
      "horizon_lock_positions", {0.0, 0.0});
    level_align_enabled_ = ParameterOr<bool>("level_align_enabled", false);
    direct_grasp_after_align_ = ParameterOr<bool>("direct_grasp_after_align", false);
    level_joint2_name_ = ParameterOr<std::string>("level_joint2_name", "joint2");
    level_joint3_name_ = ParameterOr<std::string>("level_joint3_name", "joint3");
    level_joint2_target_ = ParameterOr<double>("level_joint2_target", 1.35);
    level_joint3_target_ = ParameterOr<double>("level_joint3_target", -0.18);
    level_joint5_target_ = ParameterOr<double>("level_joint5_target", -1.10);
    level_joint2_speed_ = ParameterOr<double>("level_joint2_speed", 0.08);
    level_joint3_speed_ = ParameterOr<double>("level_joint3_speed", 0.30);
    level_joint5_speed_ = ParameterOr<double>("level_joint5_speed", 0.36);
    level_target_lost_joint5_speed_ = ParameterOr<double>(
      "level_target_lost_joint5_speed", 0.40);
    level_joint2_vertical_sign_ = ParameterOr<double>("level_joint2_vertical_sign", 1.0);
    level_joint3_vertical_sign_ = ParameterOr<double>("level_joint3_vertical_sign", -1.0);
    level_joint23_vertical_kp_ = ParameterOr<double>("level_joint23_vertical_kp", 0.00015);
    final_j5_negative_min_ = ParameterOr<double>("final_j5_negative_min", -1.18);
    final_j5_negative_max_ = ParameterOr<double>("final_j5_negative_max", -0.95);

    horizontal_error_sign_ = ParameterOr<double>("horizontal_error_sign", 1.0);
    vertical_error_sign_ = ParameterOr<double>("vertical_error_sign", 1.0);

    horizontal_search_min_ = ParameterOr<double>("horizontal_search_min", -0.90);
    horizontal_search_max_ = ParameterOr<double>("horizontal_search_max", 0.90);
    vertical_search_min_ = ParameterOr<double>("vertical_search_min", -0.60);
    vertical_search_max_ = ParameterOr<double>("vertical_search_max", 0.95);
    search_bounds_relative_to_start_ = ParameterOr<bool>(
      "search_bounds_relative_to_start", false);
    full_search_horizontal_range_ = ParameterOr<double>(
      "full_search_horizontal_range", 0.12);
    full_search_vertical_range_ = ParameterOr<double>(
      "full_search_vertical_range", 0.10);
    local_horizontal_range_ = ParameterOr<double>("local_horizontal_range", 0.08);
    local_vertical_range_ = ParameterOr<double>("local_vertical_range", 0.06);
    horizontal_search_speed_ = ParameterOr<double>("horizontal_search_speed", 0.12);
    vertical_search_speed_ = ParameterOr<double>("vertical_search_speed", 0.08);
    joint_position_tolerance_ = ParameterOr<double>("joint_position_tolerance", 0.01);
    search_timeout_s_ = ParameterOr<double>("search_timeout_s", 75.0);
    local_search_timeout_s_ = ParameterOr<double>("local_search_timeout_s", 5.0);

    target_timeout_s_ = ParameterOr<double>("target_timeout_s", 0.60);
    joint_state_timeout_s_ = ParameterOr<double>("joint_state_timeout_s", 0.50);
    target_acquire_frames_ = ParameterOr<int>("target_acquire_frames", 3);
    align_stable_frames_ = ParameterOr<int>("align_stable_frames", 5);
    target_acquire_error_ratio_ = ParameterOr<double>("target_acquire_error_ratio", 0.20);
    horizontal_kp_ = ParameterOr<double>("horizontal_kp", 0.0008);
    vertical_kp_ = ParameterOr<double>("vertical_kp", 0.0008);
    align_joint_speed_limit_ = ParameterOr<double>("align_joint_speed_limit", 0.08);
    horizon_lock_kp_ = ParameterOr<double>("horizon_lock_kp", 1.0);
    horizon_lock_speed_limit_ = ParameterOr<double>("horizon_lock_speed_limit", 0.05);
    horizon_lock_tolerance_ = ParameterOr<double>("horizon_lock_tolerance", 0.02);

    control_rate_hz_ = ParameterOr<double>("control_rate_hz", 20.0);
    prepare_velocity_scaling_ = ParameterOr<double>("prepare_velocity_scaling", 0.05);
    prepare_acceleration_scaling_ = ParameterOr<double>("prepare_acceleration_scaling", 0.05);
    prepare_planning_time_s_ = ParameterOr<double>("prepare_planning_time_s", 10.0);
    prepare_execution_timeout_s_ = ParameterOr<double>("prepare_execution_timeout_s", 45.0);

    error_topic_ = ParameterOr<std::string>(
      "error_topic", "/brain_robot_vision/pixel_error");
    target_valid_topic_ = ParameterOr<std::string>(
      "target_valid_topic", "/brain_robot_vision/target_valid");
    joint_state_topic_ = ParameterOr<std::string>("joint_state_topic", "/joint_states");
    joint_command_topic_ = ParameterOr<std::string>(
      "joint_command_topic", "/servo_node/delta_joint_cmds");
    servo_status_topic_ = ParameterOr<std::string>(
      "servo_status_topic", "/servo_node/status");
    servo_start_service_ = ParameterOr<std::string>(
      "servo_start_service", "/servo_node/start_servo");
    servo_stop_service_ = ParameterOr<std::string>(
      "servo_stop_service", "/servo_node/stop_servo");
    emergency_stop_topic_ = ParameterOr<std::string>(
      "emergency_stop_topic", "/brain_robot_control/emergency_stop");
  }

  void CreateRosInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    error_subscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      error_topic_, sensor_qos,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_x_px_ = message->vector.x;
        error_y_px_ = message->vector.y;
        target_error_ratio_ = message->vector.z;
        error_received_time_ = SteadyClock::now();
        if ((state_ == ControlState::ALIGN || state_ == ControlState::LEVEL_ALIGN ||
          state_ == ControlState::FINAL_ALIGN) && target_valid_ &&
          target_error_ratio_ <= target_acquire_error_ratio_)
        {
          ++aligned_frames_;
        } else if (state_ == ControlState::ALIGN || state_ == ControlState::LEVEL_ALIGN ||
          state_ == ControlState::FINAL_ALIGN)
        {
          aligned_frames_ = 0;
        }
      });
    target_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      target_valid_topic_, sensor_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_valid_ = message->data;
        if (target_valid_) {
          ++target_valid_frames_;
          last_target_valid_time_ = SteadyClock::now();
        } else {
          target_valid_frames_ = 0;
          aligned_frames_ = 0;
        }
      });
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, sensor_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::size_t count = std::min(message->name.size(), message->position.size());
        for (std::size_t index = 0; index < count; ++index) {
          joint_positions_[message->name[index]] = message->position[index];
        }
        joint_state_time_ = SteadyClock::now();
      });
    servo_status_subscription_ = create_subscription<std_msgs::msg::Int8>(
      servo_status_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Int8::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        servo_status_seen_ = true;
        servo_status_ = message->data;
      });
    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          StopLocked(ControlState::FAULT, "EOG_OR_EXTERNAL_EMERGENCY_STOP");
        }
        cancel_requested_ = true;
        if (move_group_) {
          move_group_->stop();
        }
      });
    joint_command_publisher_ = create_publisher<control_msgs::msg::JointJog>(
      joint_command_topic_, rclcpp::QoS(10));
    arm_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/brain_robot_grasp/arm_trajectory", rclcpp::QoS(1));
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/brain_robot_visual_control/state", rclcpp::QoS(1).transient_local());
    reason_publisher_ = create_publisher<std_msgs::msg::String>(
      "/brain_robot_visual_control/reason", rclcpp::QoS(1).transient_local());
    forward_allowed_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/brain_robot_visual_control/forward_allowed", rclcpp::QoS(1).transient_local());
    search_direction_publisher_ = create_publisher<std_msgs::msg::String>(
      "/brain_robot_visual_control/search_direction", rclcpp::QoS(1).transient_local());

    servo_start_client_ = create_client<std_srvs::srv::Trigger>(servo_start_service_);
    servo_stop_client_ = create_client<std_srvs::srv::Trigger>(servo_stop_service_);
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        HandleStart(response);
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          StopLocked(ControlState::STOPPED, "USER_STOP");
        }
        cancel_requested_ = true;
        if (move_group_) {
          move_group_->stop();
        }
        response->success = true;
        response->message = "Visual motion stopped.";
      });

    const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      [this]() {ControlTick();});
  }

  void ValidateParameters()
  {
    configuration_ok_ = true;
    if (joint_names_.size() != observe_joint_positions_.size() || joint_names_.empty()) {
      RCLCPP_ERROR(
        get_logger(), "joint_names and observe_joint_positions must have equal non-zero length.");
      configuration_ok_ = false;
    }
    const auto contains_joint = [this](const std::string & name) {
        return std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end();
      };
    if (!contains_joint(horizontal_joint_) || !contains_joint(vertical_joint_) ||
      !contains_joint(level_joint2_name_) || !contains_joint(level_joint3_name_))
    {
      RCLCPP_ERROR(get_logger(), "Search joint mapping contains an unknown joint name.");
      configuration_ok_ = false;
    }
    if (horizon_joint_names_.empty() ||
      horizon_joint_names_.size() != horizon_lock_positions_.size())
    {
      RCLCPP_ERROR(
        get_logger(),
        "horizon_joint_names and horizon_lock_positions must have equal non-zero length.");
      configuration_ok_ = false;
    }
    std::vector<std::string> controlled_joints{horizontal_joint_, vertical_joint_};
    for (const auto & joint_name : horizon_joint_names_) {
      if (!contains_joint(joint_name)) {
        RCLCPP_ERROR(get_logger(), "Horizon lock contains an unknown joint name: %s.",
          joint_name.c_str());
        configuration_ok_ = false;
      }
      controlled_joints.push_back(joint_name);
    }
    std::sort(controlled_joints.begin(), controlled_joints.end());
    if (std::adjacent_find(controlled_joints.begin(), controlled_joints.end()) !=
      controlled_joints.end())
    {
      RCLCPP_ERROR(get_logger(), "Horizontal, vertical and horizon-lock joints must be distinct.");
      configuration_ok_ = false;
    }
    if (level_joint2_name_ == level_joint3_name_ || level_joint2_name_ == horizontal_joint_ ||
      level_joint3_name_ == horizontal_joint_ || level_joint2_name_ == vertical_joint_ ||
      level_joint3_name_ == vertical_joint_ || level_joint2_speed_ <= 0.0 ||
      level_joint3_speed_ <= 0.0 || level_joint5_speed_ <= 0.0 ||
      level_target_lost_joint5_speed_ <= 0.0 ||
      level_joint23_vertical_kp_ < 0.0 || final_j5_negative_min_ >= final_j5_negative_max_ ||
      final_j5_negative_max_ >= 0.0 || level_joint5_target_ < final_j5_negative_min_ ||
      level_joint5_target_ > final_j5_negative_max_)
    {
      RCLCPP_ERROR(get_logger(), "Level-align joint mapping or speed is invalid.");
      configuration_ok_ = false;
    }
    if (control_rate_hz_ <= 0.0 ||
      horizontal_search_min_ >= horizontal_search_max_ ||
      vertical_search_min_ >= vertical_search_max_ ||
      local_horizontal_range_ <= 0.0 || local_vertical_range_ <= 0.0 ||
      full_search_horizontal_range_ <= 0.0 || full_search_vertical_range_ <= 0.0 ||
      horizontal_search_speed_ <= 0.0 || vertical_search_speed_ <= 0.0 ||
      prepare_execution_timeout_s_ <= 0.0 ||
      target_acquire_error_ratio_ <= 0.0 || target_acquire_error_ratio_ > 1.0 ||
      align_stable_frames_ < 1)
    {
      RCLCPP_ERROR(
        get_logger(), "Search bounds, speeds, rate or target-acquisition ratio are invalid.");
      configuration_ok_ = false;
    }
    if (backend_ != "simulation" && backend_ != "piper") {
      RCLCPP_ERROR(get_logger(), "backend must be simulation or piper.");
      configuration_ok_ = false;
    }
  }

  void HandleStart(const std_srvs::srv::Trigger::Response::SharedPtr & response)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!configuration_ok_) {
        response->success = false;
        response->message = "Controller parameters are invalid; see the node log.";
        return;
      }
      if (!move_group_) {
        response->success = false;
        response->message = "MoveGroup interface is not ready.";
        return;
      }
      if (backend_ == "piper" && (!real_motion_enabled_ || !observe_pose_calibrated_)) {
        response->success = false;
        response->message =
          "Physical motion remains locked: calibrate the observation pose and enable real motion.";
        return;
      }
      if (state_ == ControlState::PREPARE || state_ == ControlState::SEARCH ||
        state_ == ControlState::LOCAL_SEARCH || state_ == ControlState::ALIGN ||
        state_ == ControlState::LEVEL_ALIGN || state_ == ControlState::FINAL_ALIGN)
      {
        response->success = false;
        response->message = "Visual motion is already active.";
        return;
      }
      if (!JointStateFreshLocked()) {
        response->success = false;
        response->message = "No fresh joint state; the robot controller is not ready.";
        return;
      }

      cancel_requested_ = false;
      full_search_retries_ = 0;
      target_valid_frames_ = 0;
      aligned_frames_ = 0;
      SetStateLocked(ControlState::PREPARE, "START_AUTHORIZED");
    }

    LaunchPrepareWorker();
    response->success = true;
    response->message = "PREPARE started; J2/J3 will move to the configured observation pose.";
  }

  void LaunchPrepareWorker()
  {
    std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    if (prepare_thread_.joinable()) {
      prepare_thread_.join();
    }
    prepare_thread_ = std::thread([this]() {PrepareWorker();});
  }

  void PrepareWorker()
  {
    CallServoService(servo_stop_client_, "stop", false);
    if (cancel_requested_) {
      return;
    }

    if (skip_prepare_) {
      if (!CallServoService(servo_start_client_, "start", true)) {
        FailPrepare("SERVO_START_FAILED");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancel_requested_ || state_ != ControlState::PREPARE) {
        return;
      }
      servo_status_seen_ = false;
      servo_status_ = -1;
      const auto horizontal = CurrentJointLocked(horizontal_joint_);
      const auto vertical = CurrentJointLocked(vertical_joint_);
      if (!horizontal || !vertical) {
        StopLocked(ControlState::FAULT, "SEARCH_JOINT_STATE_MISSING");
        return;
      }
      InitializeSearchLocked(false, *horizontal, *vertical);
      SetStateLocked(ControlState::SEARCH, "CURRENT_POSE_ACCEPTED");
      return;
    }

    std::map<std::string, double> target;
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      target[joint_names_[index]] = observe_joint_positions_[index];
    }

    if (backend_ == "piper") {
      // The real adapter is the physical trajectory executor.  Do not block
      // on MoveIt's planning/execution services for the fixed simulation
      // observation pose; publish a slow, explicit current->target trajectory.
      trajectory_msgs::msg::JointTrajectory trajectory;
      trajectory.joint_names = joint_names_;
      trajectory.points.resize(2);
      double max_delta = 0.0;
      bool missing_joint_state = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (std::size_t index = 0; index < joint_names_.size(); ++index) {
          const auto current = CurrentJointLocked(joint_names_[index]);
          if (!current) {
            missing_joint_state = true;
            break;
          }
          trajectory.points[0].positions.push_back(*current);
          trajectory.points[1].positions.push_back(observe_joint_positions_[index]);
          max_delta = std::max(max_delta,
            std::abs(observe_joint_positions_[index] - *current));
        }
      }
      if (missing_joint_state) {
        FailPrepare("PREPARE_JOINT_STATE_MISSING");
        return;
      }
      const double duration_s = std::max(20.0, max_delta / 0.02);
      auto make_duration = [](double seconds) {
        builtin_interfaces::msg::Duration duration;
        duration.sec = static_cast<int32_t>(std::floor(seconds));
        duration.nanosec = static_cast<uint32_t>(
          std::llround((seconds - static_cast<double>(duration.sec)) * 1e9));
        if (duration.nanosec >= 1000000000U) {
          ++duration.sec;
          duration.nanosec -= 1000000000U;
        }
        return duration;
      };
      trajectory.points[0].time_from_start = make_duration(0.0);
      trajectory.points[1].time_from_start = make_duration(duration_s);
      arm_trajectory_publisher_->publish(trajectory);
      RCLCPP_INFO(get_logger(), "Published direct real-arm prepare trajectory (%0.1f s).", duration_s);
      const auto deadline = SteadyClock::now() +
        std::chrono::duration<double>(prepare_execution_timeout_s_);
      bool reached = false;
      while (!cancel_requested_ && SteadyClock::now() < deadline) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          reached = true;
          for (std::size_t index = 0; index < joint_names_.size(); ++index) {
            const auto current = CurrentJointLocked(joint_names_[index]);
            if (!current || std::abs(*current - observe_joint_positions_[index]) >
              joint_position_tolerance_)
            {
              reached = false;
              break;
            }
          }
        }
        if (reached) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (!reached || cancel_requested_) {
        FailPrepare(cancel_requested_ ? "PREPARE_CANCELLED" : "PREPARE_EXECUTION_FAILED");
        return;
      }
    } else {
      const auto move_group = move_group_;
      if (!move_group) {
        FailPrepare("MOVE_GROUP_NOT_READY");
        return;
      }
      move_group->setStartStateToCurrentState();
      move_group->setPlanningTime(prepare_planning_time_s_);
      move_group->setMaxVelocityScalingFactor(prepare_velocity_scaling_);
      move_group->setMaxAccelerationScalingFactor(prepare_acceleration_scaling_);
      if (!move_group->setJointValueTarget(target)) {
        FailPrepare("OBSERVE_POSE_REJECTED");
        return;
      }
      MoveGroupInterface::Plan plan;
      const bool planned = static_cast<bool>(move_group->plan(plan));
      if (!planned || cancel_requested_) {
        FailPrepare(cancel_requested_ ? "PREPARE_CANCELLED" : "PREPARE_PLAN_FAILED");
        return;
      }
      RCLCPP_INFO(get_logger(), "PREPARE plan succeeded with %zu points.",
        plan.trajectory_.joint_trajectory.points.size());
      if (!execute_prepare_) {
        FailPrepare("PREPARE_PLAN_ONLY_COMPLETE");
        return;
      }
      if (!static_cast<bool>(move_group->execute(plan)) || cancel_requested_) {
        FailPrepare(cancel_requested_ ? "PREPARE_CANCELLED" : "PREPARE_EXECUTION_FAILED");
        return;
      }
    }

    if (!CallServoService(servo_start_client_, "start", true)) {
      FailPrepare("SERVO_START_FAILED");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancel_requested_ || state_ != ControlState::PREPARE) {
        return;
      }
      servo_status_seen_ = false;
      servo_status_ = -1;
      const auto horizontal = CurrentJointLocked(horizontal_joint_);
      const auto vertical = CurrentJointLocked(vertical_joint_);
      if (!horizontal || !vertical) {
        StopLocked(ControlState::FAULT, "SEARCH_JOINT_STATE_MISSING");
        return;
      }
      InitializeSearchLocked(false, *horizontal, *vertical);
      SetStateLocked(ControlState::SEARCH, "OBSERVE_POSE_REACHED");
    }
  }

  bool CallServoService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const char * action,
    bool require_success)
  {
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Servo %s service is unavailable.", action);
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Servo %s service timed out.", action);
      return false;
    }
    const auto response = future.get();
    if (require_success && !response->success) {
      RCLCPP_ERROR(
        get_logger(), "Servo %s request failed: %s", action, response->message.c_str());
      return false;
    }
    return true;
  }

  void FailPrepare(const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ControlState::PREPARE) {
      StopLocked(
        reason == "PREPARE_PLAN_ONLY_COMPLETE" ? ControlState::STOPPED : ControlState::FAULT,
        reason);
    }
  }

  void ControlTick()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!IsMotionState(state_)) {
        return;
      }
      if (state_ == ControlState::PREPARE) {
        return;
      }
      if (!JointStateFreshLocked()) {
        StopLocked(ControlState::FAULT, "JOINT_STATE_TIMEOUT");
        return;
      }
      if (servo_status_seen_ && (servo_status_ == 2 || servo_status_ == 4 || servo_status_ == 5)) {
        StopLocked(ControlState::FAULT, "MOVEIT_SERVO_HARD_STOP_" + std::to_string(servo_status_));
        return;
      }

      switch (state_) {
        case ControlState::SEARCH:
          HandleSearchLocked(false);
          break;
        case ControlState::LOCAL_SEARCH:
          HandleSearchLocked(true);
          break;
        case ControlState::ALIGN:
          HandleAlignLocked();
          break;
        case ControlState::LEVEL_ALIGN:
          HandleLevelAlignLocked();
          break;
        case ControlState::LEVEL_RECOVERY:
          HandleLevelRecoveryLocked();
          break;
        case ControlState::FINAL_ALIGN:
          HandleFinalAlignLocked();
          break;
        default:
          break;
      }
    }
  }

  void HandleSearchLocked(bool local)
  {
    if (TargetFreshLocked() && target_valid_frames_ >= target_acquire_frames_) {
      PublishZeroLocked();
      aligned_frames_ = 0;
      PublishControlDetailLocked("PHASE=ALIGN J1/J5=VISION_CENTER");
      SetStateLocked(
        ControlState::ALIGN,
        local ? "TARGET_REACQUIRED" : "TARGET_ACQUIRED");
      return;
    }

    const double timeout = local ? local_search_timeout_s_ : search_timeout_s_;
    const double elapsed = std::chrono::duration<double>(SteadyClock::now() - search_start_time_).count();
    const bool search_complete = elapsed >= timeout || StepSearchLocked();
    if (!search_complete) {
      return;
    }

    PublishZeroLocked();
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    if (!horizontal || !vertical) {
      StopLocked(ControlState::FAULT, "SEARCH_JOINT_STATE_MISSING");
      return;
    }
    if (local) {
      // Match the simulation behavior: a failed local scan expands directly
      // into the full scan from the current pose; never reset the arm here.
      InitializeSearchLocked(false, *horizontal, *vertical);
      SetStateLocked(ControlState::SEARCH, "LOCAL_SEARCH_FAILED_CONTINUE_FULL_SCAN");
    } else {
      // Keep searching continuously.  A completed sweep starts another one
      // from the current pose instead of returning to PREPARE.
      InitializeSearchLocked(false, *horizontal, *vertical);
      SetStateLocked(ControlState::SEARCH, "FULL_SEARCH_CYCLE_RESTART");
    }
  }

  bool StepSearchLocked()
  {
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    if (!horizontal || !vertical) {
      StopLocked(ControlState::FAULT, "SEARCH_JOINT_STATE_MISSING");
      return false;
    }

    if (search_is_local_) {
      StepLocalSearchLocked(*horizontal, *vertical);
    } else {
      StepFullSearchLocked(*horizontal, *vertical);
    }
    return false;
  }

  void StepLocalSearchLocked(double horizontal, double vertical)
  {
    double horizontal_target = search_horizontal_direction_ > 0 ?
      search_horizontal_limit_max_ : search_horizontal_limit_min_;
    double vertical_target = search_vertical_direction_ > 0 ?
      search_vertical_limit_max_ : search_vertical_limit_min_;
    bool direction_changed = false;

    if (Reached(horizontal, horizontal_target)) {
      search_horizontal_direction_ *= -1;
      horizontal_target = search_horizontal_direction_ > 0 ?
        search_horizontal_limit_max_ : search_horizontal_limit_min_;
      direction_changed = true;
    }
    if (Reached(vertical, vertical_target)) {
      search_vertical_direction_ *= -1;
      vertical_target = search_vertical_direction_ > 0 ?
        search_vertical_limit_max_ : search_vertical_limit_min_;
      direction_changed = true;
    }

    if (direction_changed) {
      PublishSearchDirectionLocked();
    }
    PublishJointLocked(
      VelocityToward(horizontal, horizontal_target, horizontal_search_speed_),
      VelocityToward(vertical, vertical_target, vertical_search_speed_));
  }

  void StepFullSearchLocked(double horizontal, double vertical)
  {
    double horizontal_target = search_horizontal_direction_ > 0 ?
      search_horizontal_limit_max_ : search_horizontal_limit_min_;
    bool horizontal_boundary_hit = false;
    int horizontal_boundary_direction = 0;

    if (Reached(horizontal, horizontal_target)) {
      horizontal_boundary_hit = true;
      horizontal_boundary_direction = search_horizontal_direction_;
      search_horizontal_direction_ *= -1;
      horizontal_target = search_horizontal_direction_ > 0 ?
        search_horizontal_limit_max_ : search_horizontal_limit_min_;
    }

    double vertical_velocity = 0.0;
    bool phase_changed = false;
    switch (full_search_phase_) {
      case FullSearchPhase::DESCEND:
        search_vertical_direction_ = 1;
        if (Reached(vertical, search_vertical_limit_max_)) {
          full_search_phase_ = FullSearchPhase::BOTTOM_SCAN;
          search_vertical_direction_ = 0;
          ResetFullSearchCoverageLocked();
          phase_changed = true;
        } else {
          vertical_velocity = VelocityToward(
            vertical, search_vertical_limit_max_, vertical_search_speed_);
        }
        break;

      case FullSearchPhase::BOTTOM_SCAN:
        search_vertical_direction_ = 0;
        RecordFullSearchBoundaryLocked(
          horizontal_boundary_hit, horizontal_boundary_direction);
        if (FullSearchCoverageCompleteLocked()) {
          full_search_phase_ = FullSearchPhase::ASCEND;
          search_vertical_direction_ = -1;
          ResetFullSearchCoverageLocked();
          vertical_velocity = VelocityToward(
            vertical, search_vertical_limit_min_, vertical_search_speed_);
          phase_changed = true;
        }
        break;

      case FullSearchPhase::ASCEND:
        search_vertical_direction_ = -1;
        if (Reached(vertical, search_vertical_limit_min_)) {
          full_search_phase_ = FullSearchPhase::TOP_SCAN;
          search_vertical_direction_ = 0;
          ResetFullSearchCoverageLocked();
          phase_changed = true;
        } else {
          vertical_velocity = VelocityToward(
            vertical, search_vertical_limit_min_, vertical_search_speed_);
        }
        break;

      case FullSearchPhase::TOP_SCAN:
        search_vertical_direction_ = 0;
        RecordFullSearchBoundaryLocked(
          horizontal_boundary_hit, horizontal_boundary_direction);
        if (FullSearchCoverageCompleteLocked()) {
          full_search_phase_ = FullSearchPhase::DESCEND;
          search_vertical_direction_ = 1;
          ResetFullSearchCoverageLocked();
          vertical_velocity = VelocityToward(
            vertical, search_vertical_limit_max_, vertical_search_speed_);
          phase_changed = true;
        }
        break;
    }

    if (horizontal_boundary_hit || phase_changed) {
      PublishSearchDirectionLocked();
    }
    PublishJointLocked(
      VelocityToward(horizontal, horizontal_target, horizontal_search_speed_),
      vertical_velocity);
  }

  void RecordFullSearchBoundaryLocked(bool boundary_hit, int boundary_direction)
  {
    if (!boundary_hit) {
      return;
    }
    if (boundary_direction > 0) {
      full_search_right_seen_ = true;
    } else if (boundary_direction < 0) {
      full_search_left_seen_ = true;
    }
  }

  bool FullSearchCoverageCompleteLocked() const
  {
    return full_search_left_seen_ && full_search_right_seen_;
  }

  void ResetFullSearchCoverageLocked()
  {
    full_search_left_seen_ = false;
    full_search_right_seen_ = false;
  }

  void HandleAlignLocked()
  {
    if (!TargetFreshLocked()) {
      BeginLocalSearchLocked("TARGET_LOST_DURING_ALIGN");
      return;
    }
    const double horizontal_velocity = std::clamp(
      horizontal_error_sign_ * horizontal_kp_ * error_x_px_,
      -align_joint_speed_limit_, align_joint_speed_limit_);
    const double vertical_velocity = std::clamp(
      vertical_error_sign_ * vertical_kp_ * error_y_px_,
      -align_joint_speed_limit_, align_joint_speed_limit_);
    PublishJointLocked(horizontal_velocity, vertical_velocity);

    if (aligned_frames_ >= align_stable_frames_) {
      if (direct_grasp_after_align_) {
        aligned_frames_ = 0;
        PublishZeroLocked();
        RequestServoStopLocked();
        PublishControlDetailLocked("PHASE=GRASP_READY DIRECT_VISUAL_ALIGN");
        SetStateLocked(ControlState::GRASP_READY, "DIRECT_VISUAL_ALIGN_ACQUIRED");
        return;
      }
      if (!level_align_enabled_) {
        StopLocked(ControlState::FAULT, "LEVEL_ALIGN_NOT_CALIBRATED");
        return;
      }
      aligned_frames_ = 0;
      PublishControlDetailLocked(
        "PHASE=LEVEL_ALIGN J1=HOLD J2/J3=VISION J5=TO_NEGATIVE_ONLY");
      SetStateLocked(ControlState::LEVEL_ALIGN, "TARGET_ACQUIRED_LEVELING");
    }
  }

  void HandleLevelAlignLocked()
  {
    if (!TargetFreshLocked()) {
      PublishControlDetailLocked(
        "PHASE=LEVEL_RECOVERY J1=HOLD J2=UP J3=TO_ZERO J5=FAST_TO_NEGATIVE");
      SetStateLocked(ControlState::LEVEL_RECOVERY, "TARGET_LOST_DURING_LEVEL_ALIGN");
      return;
    }
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    const auto joint2 = CurrentJointLocked(level_joint2_name_);
    const auto joint3 = CurrentJointLocked(level_joint3_name_);
    if (!horizontal || !vertical || !joint2 || !joint3) {
      StopLocked(ControlState::FAULT, "LEVEL_ALIGN_JOINT_STATE_MISSING");
      return;
    }

    // During the J5 positive-to-negative flip, J1 must hold its detected
    // angle. J2/J3 are the only image-compensation joints in this phase.
    const double horizontal_velocity = 0.0;
    const double level_vertical_velocity = VelocityToward(
      *vertical, level_joint5_target_, level_joint5_speed_);
    const double shoulder_vertical_correction =
      level_joint23_vertical_kp_ * error_y_px_;
    PublishLevelJointLocked(
      horizontal_velocity,
      level_vertical_velocity,
      std::clamp(
        VelocityToward(*joint2, level_joint2_target_, level_joint2_speed_) +
        level_joint2_vertical_sign_ * shoulder_vertical_correction,
        -level_joint2_speed_, level_joint2_speed_),
      std::clamp(
        VelocityToward(*joint3, level_joint3_target_, level_joint3_speed_) +
        level_joint3_vertical_sign_ * shoulder_vertical_correction,
        -level_joint3_speed_, level_joint3_speed_));

    if (Reached(*vertical, level_joint5_target_))
    {
      aligned_frames_ = 0;
      PublishControlDetailLocked("PHASE=FINAL_ALIGN J1=VISION J2/J3=VISION J5=NEGATIVE_ONLY");
      SetStateLocked(ControlState::FINAL_ALIGN, "J5_NEGATIVE_LEVEL_REACHED");
    }
  }

  void HandleLevelRecoveryLocked()
  {
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    const auto joint2 = CurrentJointLocked(level_joint2_name_);
    const auto joint3 = CurrentJointLocked(level_joint3_name_);
    if (!horizontal || !vertical || !joint2 || !joint3) {
      StopLocked(ControlState::FAULT, "LEVEL_RECOVERY_JOINT_STATE_MISSING");
      return;
    }

    // The target is unavailable. Finish the requested safe posture transition
    // without using stale image error: hold J1, raise J2, bring J3 toward zero,
    // and complete the J5 negative flip at the faster recovery speed.
    PublishLevelJointLocked(
      0.0,
      VelocityToward(*vertical, level_joint5_target_, level_target_lost_joint5_speed_),
      VelocityToward(*joint2, level_joint2_target_, level_joint2_speed_),
      VelocityToward(*joint3, level_joint3_target_, level_joint3_speed_));

    if (!Reached(*vertical, level_joint5_target_)) {
      return;
    }
    if (TargetFreshLocked()) {
      aligned_frames_ = 0;
      PublishControlDetailLocked("PHASE=FINAL_ALIGN J1=VISION J2/J3=VISION J5=NEGATIVE_ONLY");
      SetStateLocked(ControlState::FINAL_ALIGN, "TARGET_REACQUIRED_AFTER_LEVEL_RECOVERY");
      return;
    }
    BeginLocalSearchLocked("TARGET_NOT_REACQUIRED_AFTER_LEVEL_RECOVERY");
  }

  void HandleFinalAlignLocked()
  {
    if (!TargetFreshLocked()) {
      BeginLocalSearchLocked("TARGET_LOST_DURING_FINAL_ALIGN");
      return;
    }
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    const auto joint2 = CurrentJointLocked(level_joint2_name_);
    const auto joint3 = CurrentJointLocked(level_joint3_name_);
    if (!horizontal || !vertical || !joint2 || !joint3) {
      StopLocked(ControlState::FAULT, "FINAL_ALIGN_JOINT_STATE_MISSING");
      return;
    }

    const double horizontal_velocity = std::clamp(
      horizontal_error_sign_ * horizontal_kp_ * error_x_px_,
      -align_joint_speed_limit_, align_joint_speed_limit_);
    double vertical_velocity = std::clamp(
      vertical_error_sign_ * vertical_kp_ * error_y_px_,
      -align_joint_speed_limit_, align_joint_speed_limit_);
    if ((*vertical <= final_j5_negative_min_ && vertical_velocity < 0.0) ||
      (*vertical >= final_j5_negative_max_ && vertical_velocity > 0.0))
    {
      vertical_velocity = 0.0;
    }
    const double shoulder_vertical_correction =
      level_joint23_vertical_kp_ * error_y_px_;
    PublishLevelJointLocked(
      horizontal_velocity,
      vertical_velocity,
      std::clamp(
        VelocityToward(*joint2, level_joint2_target_, level_joint2_speed_) +
        level_joint2_vertical_sign_ * shoulder_vertical_correction,
        -level_joint2_speed_, level_joint2_speed_),
      std::clamp(
        VelocityToward(*joint3, level_joint3_target_, level_joint3_speed_) +
        level_joint3_vertical_sign_ * shoulder_vertical_correction,
        -level_joint3_speed_, level_joint3_speed_));

    if (aligned_frames_ >= align_stable_frames_) {
      PublishZeroLocked();
      RequestServoStopLocked();
      PublishControlDetailLocked("PHASE=GRASP_READY SERVO=STOPPED");
      SetStateLocked(ControlState::GRASP_READY, "FINAL_NEGATIVE_POSTURE_ALIGNED");
    }
  }

  void BeginLocalSearchLocked(const std::string & reason)
  {
    PublishZeroLocked();
    const auto horizontal = CurrentJointLocked(horizontal_joint_);
    const auto vertical = CurrentJointLocked(vertical_joint_);
    if (!horizontal || !vertical) {
      StopLocked(ControlState::FAULT, "LOCAL_SEARCH_JOINT_STATE_MISSING");
      return;
    }
    target_valid_frames_ = 0;
    aligned_frames_ = 0;
    InitializeSearchLocked(true, *horizontal, *vertical);
    SetStateLocked(ControlState::LOCAL_SEARCH, reason);
  }

  void InitializeSearchLocked(bool local, double horizontal_center, double vertical_center)
  {
    search_is_local_ = local;
    if (local) {
      const double clamped_horizontal_center = std::clamp(
        horizontal_center, horizontal_search_min_, horizontal_search_max_);
      const double clamped_vertical_center = std::clamp(
        vertical_center, vertical_search_min_, vertical_search_max_);
      search_horizontal_limit_min_ = std::max(
        horizontal_search_min_, clamped_horizontal_center - local_horizontal_range_);
      search_horizontal_limit_max_ = std::min(
        horizontal_search_max_, clamped_horizontal_center + local_horizontal_range_);
      search_vertical_limit_min_ = std::max(
        vertical_search_min_, clamped_vertical_center - local_vertical_range_);
      search_vertical_limit_max_ = std::min(
        vertical_search_max_, clamped_vertical_center + local_vertical_range_);
    } else {
      if (search_bounds_relative_to_start_) {
        // Keep a real-arm full scan around the manually verified start pose.
        search_horizontal_limit_min_ = std::max(
          horizontal_search_min_, horizontal_center - full_search_horizontal_range_);
        search_horizontal_limit_max_ = std::min(
          horizontal_search_max_, horizontal_center + full_search_horizontal_range_);
        search_vertical_limit_min_ = std::max(
          vertical_search_min_, vertical_center - full_search_vertical_range_);
        search_vertical_limit_max_ = std::min(
          vertical_search_max_, vertical_center + full_search_vertical_range_);
      } else {
        search_horizontal_limit_min_ = horizontal_search_min_;
        search_horizontal_limit_max_ = horizontal_search_max_;
        search_vertical_limit_min_ = vertical_search_min_;
        search_vertical_limit_max_ = vertical_search_max_;
      }
    }
    if (local) {
      search_horizontal_direction_ =
        horizontal_center >= search_horizontal_limit_max_ ? -1 : 1;
      search_vertical_direction_ = vertical_center >= search_vertical_limit_max_ ? -1 : 1;
    } else {
      // Start a full scan toward negative J1; target alignment uses pixel error instead.
      search_horizontal_direction_ = -1;
      full_search_phase_ = FullSearchPhase::DESCEND;
      search_vertical_direction_ = 1;
      ResetFullSearchCoverageLocked();
    }
    search_start_time_ = SteadyClock::now();
    PublishSearchDirectionLocked();
  }

  void PublishSearchDirectionLocked()
  {
    std_msgs::msg::String message;
    const std::string j1_direction = search_horizontal_direction_ > 0 ?
      "POSITIVE" : "NEGATIVE";
    if (search_is_local_) {
      message.data = std::string("PHASE=LOCAL_SEARCH J1=") + j1_direction +
        " J5=" + (search_vertical_direction_ > 0 ? "DOWN" : "UP");
    } else {
      const char * j5_direction = "HOLD";
      if (search_vertical_direction_ > 0) {
        j5_direction = "DOWN";
      } else if (search_vertical_direction_ < 0) {
        j5_direction = "UP";
      } else if (full_search_phase_ == FullSearchPhase::BOTTOM_SCAN) {
        j5_direction = "HOLD_DOWN";
      } else if (full_search_phase_ == FullSearchPhase::TOP_SCAN) {
        j5_direction = "HOLD_UP";
      }
      message.data = std::string("PHASE=") + SearchPhaseName(full_search_phase_) +
        " J1=" + j1_direction + " J5=" + j5_direction;
    }
    PublishControlDetailLocked(message.data);
  }

  void PublishControlDetailLocked(const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = detail;
    search_direction_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Control detail: %s.", message.data.c_str());
  }

  void PublishJointLocked(
    double horizontal_velocity, double vertical_velocity, bool lock_horizon = true)
  {
    control_msgs::msg::JointJog command;
    command.header.stamp = now();
    command.joint_names = {horizontal_joint_, vertical_joint_};
    command.velocities = {horizontal_velocity, vertical_velocity};
    const std::size_t lock_count = std::min(
      horizon_joint_names_.size(), horizon_lock_positions_.size());
    for (std::size_t index = 0; index < lock_count; ++index) {
      command.joint_names.push_back(horizon_joint_names_[index]);
      command.velocities.push_back(lock_horizon ? HorizonLockVelocityLocked(index) : 0.0);
    }
    joint_command_publisher_->publish(command);
    PublishForwardAllowedLocked(false);
  }

  void PublishLevelJointLocked(
    double horizontal_velocity, double vertical_velocity,
    double joint2_velocity, double joint3_velocity)
  {
    control_msgs::msg::JointJog command;
    command.header.stamp = now();
    command.joint_names = {
      horizontal_joint_, vertical_joint_, level_joint2_name_, level_joint3_name_};
    command.velocities = {
      horizontal_velocity, vertical_velocity, joint2_velocity, joint3_velocity};
    const std::size_t lock_count = std::min(
      horizon_joint_names_.size(), horizon_lock_positions_.size());
    for (std::size_t index = 0; index < lock_count; ++index) {
      command.joint_names.push_back(horizon_joint_names_[index]);
      command.velocities.push_back(HorizonLockVelocityLocked(index));
    }
    joint_command_publisher_->publish(command);
    PublishForwardAllowedLocked(false);
  }

  void PublishZeroLocked()
  {
    PublishJointLocked(0.0, 0.0, false);
    PublishForwardAllowedLocked(false);
  }

  double HorizonLockVelocityLocked(std::size_t index) const
  {
    const auto current = CurrentJointLocked(horizon_joint_names_[index]);
    if (!current) {
      return 0.0;
    }
    const double error = horizon_lock_positions_[index] - *current;
    if (std::abs(error) <= horizon_lock_tolerance_) {
      return 0.0;
    }
    return std::clamp(
      horizon_lock_kp_ * error, -horizon_lock_speed_limit_, horizon_lock_speed_limit_);
  }

  double VelocityToward(double current, double target, double limit) const
  {
    const double error = target - current;
    if (std::abs(error) <= joint_position_tolerance_) {
      return 0.0;
    }
    return std::copysign(std::min(limit, std::abs(error) * 2.0), error);
  }

  bool Reached(double current, double target) const
  {
    return std::abs(current - target) <= joint_position_tolerance_;
  }

  std::optional<double> CurrentJointLocked(const std::string & name) const
  {
    const auto found = joint_positions_.find(name);
    if (found == joint_positions_.end()) {
      return std::nullopt;
    }
    return found->second;
  }

  bool TargetFreshLocked() const
  {
    return target_valid_ && Fresh(last_target_valid_time_, target_timeout_s_) &&
           Fresh(error_received_time_, target_timeout_s_);
  }

  bool JointStateFreshLocked() const
  {
    return Fresh(joint_state_time_, joint_state_timeout_s_);
  }

  bool Fresh(const SteadyTime & time, double timeout_s) const
  {
    if (time == SteadyTime{}) {
      return false;
    }
    return std::chrono::duration<double>(SteadyClock::now() - time).count() <= timeout_s;
  }

  static bool IsMotionState(ControlState state)
  {
    return state == ControlState::PREPARE || state == ControlState::SEARCH ||
           state == ControlState::LOCAL_SEARCH || state == ControlState::ALIGN ||
           state == ControlState::LEVEL_ALIGN || state == ControlState::LEVEL_RECOVERY ||
           state == ControlState::FINAL_ALIGN;
  }

  void StopLocked(ControlState final_state, const std::string & reason)
  {
    PublishZeroLocked();
    SetStateLocked(final_state, reason);
    RequestServoStopLocked();
  }

  void RequestServoStopLocked()
  {
    if (servo_stop_client_->service_is_ready()) {
      servo_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  void SetStateLocked(ControlState state, const std::string & reason)
  {
    const bool changed = state_ != state || last_reason_ != reason;
    state_ = state;
    last_reason_ = reason;
    PublishStateLocked(reason);
    if (changed) {
      RCLCPP_INFO(get_logger(), "STATE -> %s (%s)", StateName(state_), reason.c_str());
    }
  }

  void PublishStateLocked(const std::string & reason)
  {
    if (!state_publisher_ || !reason_publisher_) {
      return;
    }
    std_msgs::msg::String state_message;
    state_message.data = StateName(state_);
    state_publisher_->publish(state_message);
    std_msgs::msg::String reason_message;
    reason_message.data = reason;
    reason_publisher_->publish(reason_message);
  }

  void PublishForwardAllowedLocked(bool allowed)
  {
    std_msgs::msg::Bool message;
    message.data = allowed;
    forward_allowed_publisher_->publish(message);
  }

  std::mutex mutex_;
  std::mutex worker_mutex_;
  std::thread prepare_thread_;
  std::atomic_bool cancel_requested_{false};
  std::shared_ptr<MoveGroupInterface> move_group_;

  ControlState state_{ControlState::IDLE};
  std::string last_reason_;
  bool configuration_ok_{false};

  std::string backend_;
  bool real_motion_enabled_{false};
  bool observe_pose_calibrated_{true};
  bool execute_prepare_{true};
  bool skip_prepare_{false};
  std::vector<std::string> joint_names_;
  std::vector<double> observe_joint_positions_;
  std::string horizontal_joint_;
  std::string vertical_joint_;
  std::vector<std::string> horizon_joint_names_;
  std::vector<double> horizon_lock_positions_;
  bool level_align_enabled_{false};
  bool direct_grasp_after_align_{false};
  std::string level_joint2_name_;
  std::string level_joint3_name_;
  double level_joint2_target_{1.35};
  double level_joint3_target_{-0.18};
  double level_joint5_target_{-1.10};
  double level_joint2_speed_{0.08};
  double level_joint3_speed_{0.30};
  double level_joint5_speed_{0.36};
  double level_target_lost_joint5_speed_{0.40};
  double level_joint2_vertical_sign_{1.0};
  double level_joint3_vertical_sign_{-1.0};
  double level_joint23_vertical_kp_{0.00015};
  double final_j5_negative_min_{-1.18};
  double final_j5_negative_max_{-0.95};
  double horizontal_error_sign_{1.0};
  double vertical_error_sign_{1.0};

  double horizontal_search_min_{-0.90};
  double horizontal_search_max_{0.90};
  double vertical_search_min_{-0.60};
  double vertical_search_max_{0.95};
  bool search_bounds_relative_to_start_{false};
  double full_search_horizontal_range_{0.12};
  double full_search_vertical_range_{0.10};
  double local_horizontal_range_{0.08};
  double local_vertical_range_{0.06};
  double horizontal_search_speed_{0.12};
  double vertical_search_speed_{0.08};
  double joint_position_tolerance_{0.01};
  double search_timeout_s_{75.0};
  double local_search_timeout_s_{5.0};
  double target_timeout_s_{0.60};
  double joint_state_timeout_s_{0.50};
  int target_acquire_frames_{3};
  int align_stable_frames_{5};
  double target_acquire_error_ratio_{0.20};
  double horizontal_kp_{0.0008};
  double vertical_kp_{0.0008};
  double align_joint_speed_limit_{0.08};
  double horizon_lock_kp_{1.0};
  double horizon_lock_speed_limit_{0.05};
  double horizon_lock_tolerance_{0.02};
  double control_rate_hz_{20.0};
  double prepare_velocity_scaling_{0.05};
  double prepare_acceleration_scaling_{0.05};
  double prepare_planning_time_s_{10.0};
  double prepare_execution_timeout_s_{45.0};

  std::string error_topic_;
  std::string target_valid_topic_;
  std::string joint_state_topic_;
  std::string joint_command_topic_;
  std::string servo_status_topic_;
  std::string servo_start_service_;
  std::string servo_stop_service_;
  std::string emergency_stop_topic_;

  std::unordered_map<std::string, double> joint_positions_;
  SteadyTime joint_state_time_{};
  SteadyTime error_received_time_{};
  SteadyTime last_target_valid_time_{};
  bool target_valid_{false};
  double error_x_px_{0.0};
  double error_y_px_{0.0};
  double target_error_ratio_{1.0};
  int target_valid_frames_{0};
  int aligned_frames_{0};
  bool servo_status_seen_{false};
  int8_t servo_status_{-1};

  double search_horizontal_limit_min_{0.0};
  double search_horizontal_limit_max_{0.0};
  double search_vertical_limit_min_{0.0};
  double search_vertical_limit_max_{0.0};
  int search_horizontal_direction_{1};
  int search_vertical_direction_{1};
  bool search_is_local_{false};
  FullSearchPhase full_search_phase_{FullSearchPhase::DESCEND};
  bool full_search_left_seen_{false};
  bool full_search_right_seen_{false};
  int full_search_retries_{0};
  SteadyTime search_start_time_{};

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_valid_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_direction_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr forward_allowed_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace brain_robot_pick_place

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  const auto node = std::make_shared<brain_robot_pick_place::VisualSearchController>(options);

  try {
    const auto move_group = std::make_shared<brain_robot_pick_place::MoveGroupInterface>(node, "arm");
    node->SetMoveGroup(move_group);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Visual controller failed: %s", exception.what());
  }

  rclcpp::shutdown();
  return 0;
}
