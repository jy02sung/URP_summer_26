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

## Work Log - 2026-07-13

- Removed the fingertip lip geometry from `dual_arm.xacro`, keeping only the flat grip pads on both end effectors.
- Replaced the previous impedance-style grasp compliance block with an admittance controller:
  filtered F/T force is now integrated into Cartesian compliance position/velocity offsets,
  then mapped through arm-only Jacobian columns into left/right arm joint target offsets.
- Kept waist and head out of the compliance mapping so contact response only moves the arm joints.
- Reworked the main loop so nominal trajectory acceleration is built first, then the final joint target
  after admittance correction is passed through PD + RNEA.
- Rebuilt with `catkin_make` after the control changes.

## Work Log - 2026-07-14

- Began wrist DoF expansion from 11DoF to 15DoF by adding bilateral wrist yaw/pitch joints.
- Updated Pinocchio/control indexing consistently to:
  waist, head yaw, head pitch, left shoulder/elbow/wrist(6), right shoulder/elbow/wrist(6).
- Extended `main.cpp`, `dual_arm_function.cpp`, controller YAMLs, and `dual_arm_control.launch`
  to publish/load 15 effort controllers and to map `/dual_arm/joint_states` using the new alphabetical order.
- Added `L_wrist_yaw_joint`, `L_wrist_pitch_joint`, `R_wrist_yaw_joint`, `R_wrist_pitch_joint`
  into `dual_arm.xacro`, reparenting `L_EE_joint` / `R_EE_joint` under wrist pitch links so the F/T sensor joints remain intact.
- Regenerated `urdf/dual_arm.urdf` from xacro and rebuilt successfully with `catkin_make`.
- Gazebo test status:
  the robot spawns, all 15 controllers load, and `/dual_arm/joint_states/name` includes the four wrist joints,
  but the simulated `dual_arm` model still reports invalid dynamics (`/gazebo/model_states` twist = `nan`)
  and wrist-added startup is not yet physically stable.
- Current likely next step:
  isolate whether the instability comes from the new wrist link inertias/collision geometry
  or from the torque controller path by launching once with the wrist URDF but without `dual_arm_main` torque output.

## Work Log - 2026-07-14 Night Follow-up

- Rolled back from the unstable 15DoF wrist yaw+pitch attempt to a 13DoF yaw-only wrist variant:
  waist, head yaw, head pitch, left arm(shoulder pitch/roll/yaw, elbow, wrist yaw),
  right arm(shoulder pitch/roll/yaw, elbow, wrist yaw).
- Updated `main.cpp`, `dual_arm_function.cpp`, `dual_arm_function.h`, controller YAMLs,
  and `dual_arm_control.launch` for 13 controllers and 5-DoF-per-arm indexing.
- Confirmed with `xacro`/URDF checks that the yaw-only model parses and spawns successfully.
- Added diagnostic launch files:
  `launch/gazebo_spawn_only.launch` for pure spawn testing and
  `launch/gazebo_no_main.launch` for spawn + ros_control without `dual_arm_main`.
- Important diagnosis result:
  the current issue is not primarily caused by `dual_arm_main`.
  Even with `gazebo_spawn_only.launch` or `gazebo_no_main.launch`, the model/controller state is already inconsistent.
- Observed inconsistency:
  `/gazebo/get_joint_properties` and `/dual_arm/joint_states` disagree right after spawn.
  Example from fresh launch:
  `L_shoulder_pitch_joint` and `R_shoulder_pitch_joint` read about `1.600958 rad` from Gazebo,
  while `/dual_arm/joint_states` reports wrapped/offset values such as `7.884144`, `-4.682226`,
  elbow `-2*pi`, and wrist `+2*pi`.
- Observed startup drift before main control:
  spawn target for shoulder pitch was `0.80 rad`, but actual Gazebo joint properties moved to about `1.60 rad`
  before `dual_arm_main` startup hold became relevant.
- Because the same symptom appears without `dual_arm_main`, current first-priority suspicion is
  model/physics-side startup behavior in the modified arm/wrist chain, not the task logic.
- Existing FT sensor / fixed-joint feedback blocks were already present in the earlier version;
  the most meaningful structural change in the unstable path is the wrist-link insertion and its reparented EE chain.
- Fresh GUI session was relaunched after killing stale `roslaunch`, `gzserver`, `gzclient`, `roscore`,
  and related test processes so the current on-screen Gazebo window corresponds to the latest launch.
- Best next debugging steps:
  1. spawn with `paused:=true` and inspect pure pre-physics initial pose,
  2. unpause and identify which joint/link first deviates from the requested `-J` pose,
  3. compare the yaw-only wrist chain against the last stable no-wrist model at the URDF joint/inertia level.
