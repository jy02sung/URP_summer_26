# Wrist IK / Contact Split Design

## Goal

Implement wrist support for cube grasping without making the wrist a heavy preplanned IK target.

The intended behavior is:

- Use the arm to bring each hand near the cube.
- Keep wrist yaw near neutral during pregrasp and approach.
- After contact, allow wrist yaw to compliantly align the palm with the cube face.
- Keep transport active only after face contact is stabilized.

## Frame Split

Use two end-effector roles per arm:

- `wrist_ik_frame`
  - Purpose: IK target only
  - Parent: wrist yaw link
  - Behavior: used for gross arm positioning
- `palm_contact_frame`
  - Purpose: grasp/contact/admittance reference
  - Parent: existing palm/EE link
  - Behavior: used for cube face targets, force sensing, and contact alignment

In code:

- IK solves against `L_wrist_ik_joint` / `R_wrist_ik_joint`
- Contact/grasp logic keeps using `L_EE_joint` / `R_EE_joint`

## Control Roles

- Shoulder and elbow:
  - Move the hand near the object
- Wrist yaw:
  - Not a primary preplanned IK degree of freedom
  - Used as a small-range compliance/alignment degree of freedom after contact
- Palm contact frame:
  - Defines actual grasp face targets and transport path

## Motion Phases

1. Start from a standing posture with both arms down.
2. Tilt only the head to detect the cube pose from the head camera.
3. Raise both arms upward from the standing pose before moving toward the cube.
4. Move both arms to a high pregrasp pose above the cube.
5. Align left/right palms over the cube faces while staying high.
6. Descend vertically onto the cube faces.
7. Convert those palm contact targets into IK-frame targets using the current seed pose offset.
8. Solve arm IK on wrist IK frames.
9. When contact appears, enable wrist/arm compliance to reduce contact moment and equalize face contact.
10. Only after bilateral face contact is stable, lift and transport the cube.

## Practical Mapping Rule

The planner still reasons in palm contact space because the cube face geometry is easiest to define there.

For each waypoint:

- compute the current world offset between `wrist_ik_frame` and `palm_contact_frame`
- map desired palm target to IK target by subtracting that offset
- solve IK on the wrist IK frame

This keeps the grasp trajectory defined at the palm while preventing the wrist from being a hard global planning target.

## Why This Split Helps

- Lower IK burden than directly planning the palm pose through the wrist
- Cleaner separation between gross reaching and fine contact alignment
- Better match to the desired behavior: wrist reacts to contact instead of being fully precomputed

## Current Implementation Direction

- Add fixed IK frames under the wrist links
- Keep startup in a stable standing posture instead of spawning into a raised-arm pose
- Keep existing EE links and F/T sensors as contact references
- Use IK frames for:
  - mode 1 single-shot IK
  - mode 2 Cartesian simulation
  - vision pick pipeline waypoint IK
- Keep contact EE frames for:
  - cube-face target construction
  - admittance/contact gating
  - transport/release contact references
- Use a top-down grasp sequence in vision pick:
  - `head_scan`
  - `arm_raise`
  - `pregrasp_high`
  - `descend`
  - `squeeze`
  - `lift`
  - `transport`
  - `release`
