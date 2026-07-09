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
