# Project Instructions

- Work on the `jys` branch.
- Main ROS workspace: `~/catkin_ws`.
- Main package: `src/Dual_Arm_ROS_Simulation`.
- Target environment: Ubuntu 20.04, ROS Noetic, Gazebo, Pinocchio.
- Build from workspace root with `catkin_make`.
- After building, run `source devel/setup.bash`.

## Goal

Implement and debug humanoid upper-body control so the robot:
1. Tilts its head downward to see the ArUco marker on the cube.
2. Detects the cube pose from the head camera.
3. Moves both arms toward the cube.
4. Grasps the cube using contact feedback.
5. Lifts the cube.
6. Put the cube back down on the table.

## Control Constraints

- Keep the 11DoF model: waist, 4 left arm joints, 4 right arm joints, head yaw, head pitch.
- Do not remove the head joints just to avoid coupling.
- Do not change the current P, D gains (except the neck or head) unless there is a clear reason to change them.
- During head pitch motion, keep waist, arms, and head yaw locked unless there is a clear reason to move them.
- Arm IK should only use arm joints:
  - Left arm: joints 1, 2, 3, 4.
  - Right arm: joints 5, 6, 7, 8.
- Do not allow arm IK to use waist or head joints unless explicitly requested.
- Be careful with Pinocchio joint ordering versus ROS controller joint ordering.

## Safety

- Do not run `git reset --hard`, `git clean -fd`, or discard user changes unless explicitly asked.
- Before major edits, inspect `git status`.
- Prefer small, focused patches.
- After code changes, run `catkin_make`.
- If Gazebo or ROS nodes must be restarted, explain what was restarted.

## Simulation Testing

- Use Gazebo GUI for motion debugging because robot vibration, posture drift, and unintended waist/elbow movement must be visually inspected.
- Start simulation with:
  `roslaunch dual_arm gazebo.launch gui:=true`
- Codex may start and stop ROS/Gazebo processes for testing.
- Before starting a new simulation, check for existing ROS/Gazebo processes.
- After testing, stop processes started by Codex, including `roslaunch`, `gzserver`, `gzclient`, and temporary ROS nodes.
- Do not kill unrelated user processes unless clearly part of the current ROS/Gazebo test session.

## Work Log - 2026-07-10

- Built the workspace with `catkin_make` after code changes and sourced `devel/setup.bash` for ROS tests.
- Added ArUco detector startup to `gazebo.launch` and connected it to `/dual_arm/camera/image_raw` and `/dual_arm/camera/camera_info`.
- Added `Head_camera_optical` static TF in `gazebo.launch`; using `Head` directly as an ArUco camera frame produced incorrect world poses.
- Fixed Pinocchio versus ROS controller ordering with explicit mapping:
  ROS controller order is waist, left arm, right arm, head yaw, head pitch.
  Pinocchio order is waist, head yaw, head pitch, left arm, right arm.
- Added arm-only position IK paths using only left arm Pinocchio indices 3-6 and right arm indices 7-10.
- Changed head pitch target to +40 degrees because the URDF joint sign makes positive pitch look down at the table/cube.
- Reduced only head yaw/head pitch gains to reduce neck overshoot.
- Implemented pick-place state phases: head tilt, ArUco wait, arm approach, contact squeeze, lift, put down, release.
- Simulation validation:
  head pitch reached about +0.697 rad while waist, arms, and head yaw stayed near zero during head-only motion.
  ArUco marker became visible in the head camera and `/aruco_single/pose` published in `world` near x=0.36, z=0.95 after adding the optical frame.
  Arm approach entered squeeze phase, but contact sensors stayed empty; lift/place was not fully validated because arm IK/tracking still did not bring both end effectors onto the cube reliably.
