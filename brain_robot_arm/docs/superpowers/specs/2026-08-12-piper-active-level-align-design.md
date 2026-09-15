# PiPER active level alignment before point grasp

## Goal

Restore the approved simulation control sequence: find the cup visually, use
coordinated joint motion to put the gripper into a reachable level posture, and
only then use the RGB-D point for a low-speed Cartesian side grasp.

## State flow

`SEARCH -> ALIGN -> LEVEL_ALIGN -> FINAL_ALIGN -> GRASP_READY`.

If vision is lost while J5 crosses through zero, the controller temporarily
enters `LEVEL_RECOVERY` before either final alignment or local re-search.

- Once the search gets one valid target frame, it enters `ALIGN`. `ALIGN` uses
  J1/J5 to put the cup at the image center before the active posture change.
  Only after the configured stable image gate does it enter `LEVEL_ALIGN`.
- `LEVEL_ALIGN` holds J1 at its detected pose and makes J5 move through zero
  to its configured negative target without any visual-height correction.
  J2/J3 alone compensate image height while J4/J6 remain horizon locks. J3
  approaches its `-0.18 rad` near-zero safety target at the larger speed.
- `LEVEL_RECOVERY` is entered if vision is lost during the J5 flip. It holds
  J1, continues J2 upward, drives J3 toward its near-zero safety target, and increases the already
  faster J5 negative flip speed. At J5's strongly negative target, a reacquired target enters
  `FINAL_ALIGN`; otherwise the controller begins local re-search.
- `FINAL_ALIGN` begins only after J5 is negative. J5 may then correct image
  height, but is constrained to a strongly negative configured range; J3 keeps
  its small-angle posture while J1/J2/J3/J5 perform final alignment.
- `GRASP_READY` is emitted only when the final image gate is stable. Servo is
  stopped before arm translation.

## Grasp handoff

The executor opens the gripper and removes the simulated cup collision during
`LEVEL_ALIGN`, then waits. At `GRASP_READY` it captures the reached
`gripper_base` orientation, keeps that orientation fixed, and derives only the
translation from the RGB-D point and current camera optical forward direction.
It then plans the safe stand-off, executes the low-speed Cartesian approach,
closes, attaches, and lifts.

## Safety and reuse

The new active level alignment is enabled only by the simulation profile. The
PiPER/Gemini2 profile remains physically locked (`real_motion_enabled: false`,
`simulation_only: false`, `auto_execute: false`) and explicitly disables level
alignment until its joint targets and camera-to-gripper calibration are
measured.

## Validation

Ubuntu validation must show the state sequence above, J2/J3/J5 crossing their
configured targets, `GRASP_READY`, and then a grasp execution state. Windows
static checks alone do not prove ROS2 or Gazebo behavior.
