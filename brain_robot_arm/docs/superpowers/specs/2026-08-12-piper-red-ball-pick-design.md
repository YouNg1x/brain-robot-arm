# PiPER red-ball visual direct-grasp project

## Goal

Create an independent ROS 2 package named `brain_robot_ball_pick`. In Gazebo,
PiPER must visually locate a red spherical ball, center it with the wrist RGB-D
camera, then directly grasp and lift it. The ball has radius `0.0175 m`, half
the existing medicine-cup radius of `0.035 m`.

## Scope and reuse boundary

The package reuses the existing PiPER Gazebo, MoveIt2, wrist-camera, RGB-D
topics, and the SSVEP/EOG authorization topic boundary. It does not change the
cup project package or inherit its cup-leveling state machine.

The physical PiPER/Gemini2 profile remains motion-locked by default. A later
physical deployment must supply camera-to-gripper calibration and explicit
SSVEP/EOG authorization; simulation success is not hardware validation.

## State flow

`PREPARE -> SEARCH -> ALIGN -> GRASP_READY -> BALL_GRASP -> LIFT/HOLD`.

- `PREPARE` moves only to the configured observation posture.
- `SEARCH` scans with J1/J5 and has the same bounded local-recovery behavior
  as the existing visual search.
- `ALIGN` uses J1/J5 pixel error until the red ball is stable at image center.
- `GRASP_READY` stops visual Servo before point-motion execution.
- `BALL_GRASP` opens the gripper, obtains the RGB-D camera point, plans a safe
  pregrasp, approaches in a straight line, closes the gripper, attaches the
  simulated ball, and lifts it.

No J2/J3/J5 cup-leveling or liquid-preserving posture transition is included.

## Red-ball perception

The detector retains the existing two-range red HSV segmentation and RGB-D
median-depth measurement. It adds ball-specific filtering:

- smaller allowed contour area than the cup;
- contour circularity scoring;
- configured size range to reject large red objects;
- depth median from an eroded filled contour;
- the existing pixel-error and camera-frame 3D-point output contract.

This retains the same RGB/depth/CameraInfo remapping contract for the simulated
wrist camera and a future Gemini2 aligned-depth source.

## Package contents

- red sphere Gazebo model and a ball-specific scene manager;
- red-ball RGB-D detector;
- direct-grasp executor and Gazebo ball-follow attachment behavior;
- simulation and physical-locked YAML profiles;
- launch files and an independent one-command simulation launcher.

## Verification

1. Ubuntu builds only `brain_robot_ball_pick` successfully.
2. Gazebo contains one red sphere of radius `0.0175 m` on the table.
3. Detector publishes a valid red-ball pixel, RGB-D depth, and camera-frame
   point without selecting the cup.
4. Controller reaches `GRASP_READY`, then the ball is closed, attached, and
   lifted in Gazebo.
5. The physical profile remains auto-execution locked.
