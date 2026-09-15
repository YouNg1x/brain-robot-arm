# ub Workspace

This repository is the complete `C:\ub` workspace snapshot. The main project is under `brain_robot_arm/` and contains PiPER robotic-arm experiments, ROS 2 workspaces, camera and vision components, simulation demos, real-arm control scripts, and related design documents.

## Main directories

- `brain_robot_arm/`: robotic-arm project source, ROS 2 packages, scripts, and documentation.
- `tmp/`: workspace experiment and rendering artifacts retained from the original workspace snapshot.
- `.agents/`: workspace-local agent configuration, when present.

## Safety

Simulation, vision, and real-arm control must remain separately validated. Before operating a physical arm, verify camera extrinsics, joint limits, speed limits, workspace limits, emergency stop behavior, and explicit operator confirmation. Do not unlock an unattended full grasp sequence based only on simulation success.

## Repository scope

Git metadata, local caches, Python bytecode, ROS 2 build output, runtime logs, and the local Conda installer are excluded. Third-party source and license files under `brain_robot_arm/vendor/` are retained with the workspace snapshot.
