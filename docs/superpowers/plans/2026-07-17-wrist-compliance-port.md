# Wrist Compliance Port (from `jys` branch) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed-mount grip disc pad (which is only calibrated for one nominal arm configuration and visibly misaligns whenever waist/approach angle differs) with a real 1-DOF wrist yaw joint per arm that passively self-aligns to the box face via low PD stiffness + F/T-torque-gated state machine, ported from the `jys` branch's working implementation but re-integrated onto `admittance-control`'s existing stacked-Jacobian IK and K=0 admittance control (which `jys` does not have in the same form).

**Architecture:** Add `L_wrist_yaw_joint`/`R_wrist_yaw_joint` (revolute, axis z, ±0.9 rad, soft PD) between elbow and EE in the URDF, with a separate `wrist_ik_frame` (IK target, Jacobian column zeroed so IK never "spends" the wrist) and `grip_frame` (contact/admittance reference, replaces the STL-mesh+disc collision with a single flat box palm). Existing grasp geometry (`grasp_offset`, `objL/R`, `transportL/R`, standoff logic) is untouched — a one-line offset-remap lambda converts those existing targets into `wrist_ik_frame` targets right before calling `SolveIK_Position`. `SolveIK_Position`'s existing 9-column stacked Waist+arms Jacobian (which already solves the "waist collision" problem per `DEFENSE_GUIDE.md`) is widened to 11 columns with the 2 new wrist columns zeroed — **`jys`'s per-arm-split IK function is deliberately NOT ported**, because it solves each arm independently without Waist, which would regress the coupled-Jacobian fix this branch already has.

**Tech Stack:** ROS Noetic, Gazebo 11, pinocchio (C++ and Python 3 bindings, already installed), Eigen, xacro/URDF, ros_control (position/effort controllers).

## Global Constraints

- `DoF` goes from 11 to 13. Every array/vector currently sized `[DoF]` in `dual_arm_function.h` must grow to 13 entries, in this exact order (to be confirmed empirically in Task 1, not assumed): `0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:L_wy 8:R_sp 9:R_sr 10:R_sy 11:R_e 12:R_wy`.
- Do not touch `grasp_offset` (0.065), `MARKER_TO_BOX_CENTER` (0.0755), `STANDOFF_Y` (0.15), or any of the box-size geometry from the prior commits (`1605682`, `553d9fb`) — this port must not regress the 15cm-box tuning already committed.
- Do not port `jys`'s per-arm-split `SolveIK_Position` (separate `JL`/`JR`, no Waist column). Keep this branch's single stacked 6x`DoF` Jacobian, only widen the column count and zero the 2 wrist columns.
- New controller topics must be **appended** as `joint12`/`joint13` in the yaml configs, not inserted mid-sequence — existing `joint1`..`joint11` names/PIDs stay byte-identical so nothing else (launch files, any RViz/rqt configs) that references them by number breaks.
- Every task that touches `urdf/dual_arm.xacro` or the pinocchio-facing C++ must be verified **offline first** (pinocchio Python or a small throwaway C++/pinocchio check), before any `catkin_make`/Gazebo run — this matches the project's own established practice (see `RETRY_FROM_88PCT.md`) and is much faster to iterate.
- Follow this repo's commit convention from `CLAUDE.md`: commit after each task once verified, structured 4-section message (배경/문제, 검토한 대안과 선택 이유, 구현 내용, 결과/검증), no need to ask permission per-commit (already pre-approved for this repo).

---

## Task 1: Add the wrist joint chain to the URDF, verify DoF ordering offline

**Files:**
- Modify: `urdf/dual_arm.xacro` (L arm block ~lines 329-453, R arm block ~lines 672-796 — exact line numbers will have shifted from the box-resize commit, re-locate via `grep -n 'L_elbow_joint\|L_EE_joint'`)
- Create (scratch, not committed): a throwaway check script under `/tmp` to load the URDF via pinocchio and print `model.names`

**Interfaces:**
- Produces: `L_wrist_yaw_joint`, `R_wrist_yaw_joint` (revolute, controllable), `L_wrist_ik_joint`/`R_wrist_ik_joint` (fixed, frame name used by Task 3's IK call), `L_grip_joint`/`R_grip_joint` (fixed, frame name used by Task 4's contact/admittance code). `L_EE_joint`/`R_EE_joint` keep their current name/semantics (nominal reach target) — do not rename them, `main.cpp`'s existing `objL/R`/`transportL/R` math targets this frame by name via `model.getFrameId("L_EE_joint")` and must keep working unmodified.

- [ ] **Step 1: Insert the L-arm wrist chain**

Find the existing `L_elbow_joint` → `L_EE` fixed-joint block (currently `L_elbow_joint` connects directly to `L_EE` via a `-0.3` z offset). Replace the direct connection with the chain `L_elbow → L_wrist_yaw → (L_wrist_ik_frame, L_EE)`, and add `L_grip_frame` as a child of `L_EE` at the new palm collision's center. Use this exact XML (adapt `L_EE`'s existing `<visual>`/`<inertial>` blocks — keep the existing STL `<visual>` for looks, but drop the STL `<collision>` and the grip-disc `<visual>`/`<collision>` entirely, replacing collision with one flat box):

```xml
<link name="L_wrist_yaw">
  <inertial>
    <origin xyz="0 0 -0.015" rpy="0 0 0" />
    <mass value="0.5" />
    <inertia ixx="2.5E-03" ixy="0" ixz="0" iyy="2.5E-03" iyz="0" izz="1.0E-03" />
  </inertial>
  <visual>
    <origin xyz="0 0 -0.015" rpy="0 0 0" />
    <geometry><cylinder radius="0.017" length="0.03" /></geometry>
    <material name="wrist_dark"><color rgba="0.18 0.18 0.18 1" /></material>
  </visual>
  <!-- No collision on this link: STL mesh + wrist + pad collisions overlapping
       caused Gazebo startup instability in jys's testing. Keep collision only
       on L_EE below. -->
</link>
<joint name="L_wrist_yaw_joint" type="revolute">
  <origin xyz="0 0 -0.15" rpy="0 0 0" />
  <parent link="L_elbow" />
  <child link="L_wrist_yaw" />
  <axis xyz="0 0 1" />
  <dynamics damping="1" />
  <limit lower="-0.9" upper="0.9" effort="12" velocity="4.0" />
</joint>

<!-- Fixed IK-only frame: give it a tiny nonzero inertial or ODE's fixed-joint
     lumping can become ill-conditioned with a controlled joint upstream. -->
<link name="L_wrist_ik_frame">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="0.001" />
    <inertia ixx="1.0E-06" ixy="0" ixz="0" iyy="1.0E-06" iyz="0" izz="1.0E-06" />
  </inertial>
</link>
<joint name="L_wrist_ik_joint" type="fixed">
  <origin xyz="0 0 -0.10" rpy="0 0 0" />
  <parent link="L_wrist_yaw" />
  <child link="L_wrist_ik_frame" />
</joint>

<link name="L_EE">
  <inertial>
    <origin xyz="0 0 -0.003514" rpy="0 0 0" />
    <mass value="0.15" />
    <inertia ixx="5.5E-05" ixy="0" ixz="0" iyy="5.5E-05" iyz="0" izz="6.4E-05" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry><mesh filename="package://dual_arm/meshes/L_EE.STL" /></geometry>
    <material name=""><color rgba="0.298039215686275 0.298039215686275 0.298039215686275 1" /></material>
  </visual>
  <!-- Single flat palm collision replaces the old STL-mesh + disc-pad combo.
       Offset/size copied from jys (0.090 x 0.025 x 0.090 box, 3.75cm out in -Y,
       0.5cm down) -- this is a placeholder mount pose to be re-tuned in Task 6
       against this branch's actual arm geometry, same way the old disc pad
       needed 3 rounds of offline recomputation. -->
  <collision name="L_EE_palm_collision">
    <origin xyz="0 -0.0375 -0.005" rpy="0 0 0" />
    <geometry><box size="0.090 0.025 0.090" /></geometry>
  </collision>
  <visual name="L_EE_grip_pad_visual">
    <origin xyz="0 -0.0375 -0.005" rpy="0 0 0" />
    <geometry><box size="0.090 0.025 0.090" /></geometry>
    <material name="grip_pad_visual_dark"><color rgba="0.12 0.12 0.12 1" /></material>
  </visual>
</link>
<joint name="L_EE_joint" type="fixed">
  <origin xyz="0 0 -0.15" rpy="0 0 0" />
  <parent link="L_wrist_yaw" />
  <child link="L_EE" />
</joint>

<link name="L_grip_frame" />
<joint name="L_grip_joint" type="fixed">
  <origin xyz="0 -0.0375 -0.005" rpy="0 0 0" />
  <parent link="L_EE" />
  <child link="L_grip_frame" />
</joint>
```

Note `L_wrist_yaw`/`L_wrist_ik_frame`/`L_EE` are now all fixed-offset siblings hanging off `L_wrist_yaw` at different z-depths (`-0.10` for the IK frame, `-0.15` for EE) — they are NOT coincident. This is intentional (see Task 4's offset-remap lambda).

- [ ] **Step 2: Mirror the same chain for the R arm**

Apply the identical structure to the `R_elbow_joint`/`R_EE` block, with these mirrored values (swap the y-sign on the palm offset, same as the existing box-face symmetry elsewhere in this file):

```xml
<link name="R_wrist_yaw"> ... (identical to L, just named R_wrist_yaw) </link>
<joint name="R_wrist_yaw_joint" type="revolute">
  <origin xyz="0 0 -0.15" rpy="0 0 0" />
  <parent link="R_elbow" />
  <child link="R_wrist_yaw" />
  <axis xyz="0 0 1" />
  <dynamics damping="1" />
  <limit lower="-0.9" upper="0.9" effort="12" velocity="4.0" />
</joint>
<link name="R_wrist_ik_frame"> ... </link>
<joint name="R_wrist_ik_joint" type="fixed">
  <origin xyz="0 0 -0.10" rpy="0 0 0" />
  <parent link="R_wrist_yaw" />
  <child link="R_wrist_ik_frame" />
</joint>
<link name="R_EE">
  ... (keep existing inertial/visual)
  <collision name="R_EE_palm_collision">
    <origin xyz="0 0.0375 -0.005" rpy="0 0 0" />
    <geometry><box size="0.090 0.025 0.090" /></geometry>
  </collision>
  <visual name="R_EE_grip_pad_visual">
    <origin xyz="0 0.0375 -0.005" rpy="0 0 0" />
    <geometry><box size="0.090 0.025 0.090" /></geometry>
    <material name="grip_pad_visual_dark"><color rgba="0.12 0.12 0.12 1" /></material>
  </visual>
</link>
<joint name="R_EE_joint" type="fixed">
  <origin xyz="0 0 -0.15" rpy="0 0 0" />
  <parent link="R_wrist_yaw" />
  <child link="R_EE" />
</joint>
<link name="R_grip_frame" />
<joint name="R_grip_joint" type="fixed">
  <origin xyz="0 0.0375 -0.005" rpy="0 0 0" />
  <parent link="R_EE" />
  <child link="R_grip_frame" />
</joint>
```

- [ ] **Step 3: Add `<gazebo reference="...">` contact tuning for the new palm collision**

The existing `<gazebo reference="L_EE">`/`<gazebo reference="R_EE">` blocks (mu1/mu2=1.2, kp=100000, kd=100, minDepth=0.003) already apply link-wide to whatever collisions exist on `L_EE`/`R_EE` — since Step 1/2 replaced the collisions but kept the link name `L_EE`/`R_EE`, **no change needed here**. Confirm by grepping:

```bash
grep -n 'gazebo reference="L_EE"\|gazebo reference="R_EE"' urdf/dual_arm.xacro
```//

Expect: both blocks still present, unchanged.

- [ ] **Step 4: Regenerate the flat `dual_arm.urdf` from the xacro**

```bash
cd /home/jungmin/friend_ws_88pct/src/Dual_Arm_ROS_Simulation
xacro urdf/dual_arm.xacro > urdf/dual_arm.urdf
```

- [ ] **Step 5: Offline-verify the URDF loads and confirm the actual DoF order**

```bash
python3 - <<'EOF'
import pinocchio as pin
model = pin.buildModelFromUrdf('urdf/dual_arm.urdf')
print("nq", model.nq, "nv", model.nv)
for i, name in enumerate(model.names):
    print(i, name)
for frame in ["L_wrist_ik_joint", "R_wrist_ik_joint", "L_EE_joint", "R_EE_joint", "L_grip_joint", "R_grip_joint"]:
    print(frame, "->", model.getFrameId(frame))
EOF
```

Expected: `nq`/`nv` = 13. Confirm the printed joint order matches `0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:L_wy 8:R_sp 9:R_sr 10:R_sy 11:R_e 12:R_wy` (this is what `jys` observed and what branch-grouped URDF traversal predicts, but **do not assume** — read the actual printed order and use it, adjusting Task 2's array indices if it differs). All 6 `getFrameId` calls must return valid non-huge indices (pinocchio returns `model.nframes` — an out-of-range sentinel — if the name doesn't exist; confirm each result is `< model.nframes`).

- [ ] **Step 6: Commit**

```bash
cd /home/jungmin/friend_ws_88pct/src/Dual_Arm_ROS_Simulation
git add urdf/dual_arm.xacro urdf/dual_arm.urdf
git commit -m "$(cat <<'EOF'
feat: L/R 손목 yaw 관절 추가, 그립 패드를 원판에서 평면 박스로 교체

배경/문제:
고정 원판 패드는 오프라인 스크립트로 계산한 rpy가 "그 순간 그 자세 하나"에만
맞는 상수값이라, waist 각도나 접근 자세가 달라지면 다시 어긋난다(사용자 확인:
"지금 패드가 완벽히 평행하지 않아"). 손목 관절 없이는 손 방향이 IK task에
아예 안 들어가 있어 근본적으로 고칠 수 없는 구조였음.

검토한 대안과 선택 이유:
패드 제거+구형 접촉만으로 마찰력을 올려 대응하는 안은 기각(점 접촉은 접촉
법선 축 둘레 토크를 저항할 모멘트 암이 없어 mu를 올려도 회전을 못 막음,
dual_arm.xacro:386-396에 이미 이 실패가 기록돼 있음). 별도 브랜치 jys에
이미 작동하는 손목 컴플라이언스 구현이 있어(WRIST_COMPLIANCE_DESIGN.md,
merge-base가 초기 커밋 수준이라 diff/cherry-pick 불가, 개념만 이식) 그
설계를 이 브랜치의 기존 구조 위에 재구현하기로 함.

구현 내용:
urdf/dual_arm.xacro의 L_elbow/R_elbow -> L_EE/R_EE 직결을 L_elbow ->
L_wrist_yaw(revolute, axis z, ±0.9rad) -> {L_wrist_ik_frame(고정, IK 전용),
L_EE(고정, 기존 grasp_offset 수학이 타겟하는 nominal 프레임 이름 유지)}
구조로 변경. L_EE의 충돌 형상을 기존 STL mesh+원판 조합에서 평면 박스
하나(0.090x0.025x0.090)로 교체하고 L_grip_frame(접촉/admittance 기준)을
그 중심에 추가. R arm도 동일 대칭 구조로 미러링. mu1/mu2/kp/kd/minDepth는
링크명(L_EE/R_EE) 기준으로 이미 걸려있어 추가 수정 불필요.

결과/검증:
xacro로 urdf 재생성 후 pinocchio로 로드, nq/nv=13 확인, 실제 관절 순서를
출력해 0:Waist 1:Head_yaw 2:Head_pitch 3~7:L arm(wrist 포함) 8~12:R
arm(wrist 포함) 순서임을 확인. L_wrist_ik_joint/R_wrist_ik_joint/
L_EE_joint/R_EE_joint/L_grip_joint/R_grip_joint 전부 getFrameId로 유효한
프레임임을 확인. Gazebo 실행 검증은 다음 태스크(컨트롤러/트랜스미션 추가)
이후로 미룸 - 지금 상태로는 새 관절에 액추에이터가 없어 아직 못 띄움.
EOF
)"
```

---

## Task 2: Add transmissions + controller config for the 2 new joints

**Files:**
- Modify: `urdf/dual_arm.xacro` (transmission section, search `<transmission name="tran`)
- Modify: `config/dual_arm_positioncontrol.yaml`
- Modify: `config/dual_arm_effortcontrol.yaml`

**Interfaces:**
- Consumes: `L_wrist_yaw_joint`/`R_wrist_yaw_joint` from Task 1.
- Produces: ros_control interfaces named `joint12_position_controller`/`joint12_effort_controller` (→ `L_wrist_yaw_joint`) and `joint13_position_controller`/`joint13_effort_controller` (→ `R_wrist_yaw_joint`).

- [ ] **Step 1: Add transmissions**

Find the existing Waist transmission block pattern (`<transmission name="tran1">` for `PositionJointInterface`, a second block later for `EffortJointInterface`). Add two new `SimpleTransmission` pairs (Position + Effort) for `L_wrist_yaw_joint` and `R_wrist_yaw_joint`, following the exact same pattern as the existing arm-joint transmissions (`mechanicalReduction=1`, matching e.g. `L_elbow_joint`'s transmission, not Waist's `50`):

```xml
<!-- Position Control -->
<transmission name="tran_L_wrist">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="L_wrist_yaw_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor_L_wrist">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
<transmission name="tran_R_wrist">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="R_wrist_yaw_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor_R_wrist">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

Add the matching `EffortJointInterface` pair in the effort-control transmission section further down (same joints, `hardwareInterface` swapped to `EffortJointInterface`).

- [ ] **Step 2: Append controllers to both yaml configs**

In `config/dual_arm_positioncontrol.yaml`, after the existing `joint11_position_controller` (Head_pitch) block, append:

```yaml
  joint12_position_controller:
    type: position_controllers/JointPositionController
    joint: L_wrist_yaw_joint
    pid: {p: 40.0, i: 0.0, d: 8.0}
  joint13_position_controller:
    type: position_controllers/JointPositionController
    joint: R_wrist_yaw_joint
    pid: {p: 40.0, i: 0.0, d: 8.0}
```

In `config/dual_arm_effortcontrol.yaml`, after `joint11_effort_controller`, append the effort equivalent:

```yaml
  joint12_effort_controller:
    type: effort_controllers/JointEffortController
    joint: L_wrist_yaw_joint
    pid: {p: 40.0, i: 0.0, d: 8.0}
  joint13_effort_controller:
    type: effort_controllers/JointEffortController
    joint: R_wrist_yaw_joint
    pid: {p: 40.0, i: 0.0, d: 8.0}
```

(PID `p: 40.0` mirrors the soft `Kp=40` from `jys`'s C++ side, ported to Task 3's `Kp[DoF]` array too — keep these two in sync.)

- [ ] **Step 3: Check the joint spawner list picks up the 2 new controllers**

```bash
grep -n "controller_spawner\|joint1_\|args=" launch/gazebo.launch
```

Confirm the `controller_manager/spawner` node's `args` either lists controllers explicitly (in which case add `joint12_position_controller joint12_effort_controller joint13_position_controller joint13_effort_controller` — pick whichever interface this launch file's `command_mode`/config actually loads, matching the existing 11) or loads them by wildcard/all-in-namespace (in which case no launch file change is needed — just note which case it is).

- [ ] **Step 4: Offline-verify yaml/xacro consistency**

```bash
python3 -c "import yaml; d = yaml.safe_load(open('config/dual_arm_positioncontrol.yaml')); print(sorted(d['dual_arm'].keys()))"
python3 -c "import yaml; d = yaml.safe_load(open('config/dual_arm_effortcontrol.yaml')); print(sorted(d['dual_arm'].keys()))"
```

Expected: both list `joint1_..._controller` through `joint13_..._controller` plus `joint_state_controller`, 14 entries total.

- [ ] **Step 5: Commit**

```bash
git add urdf/dual_arm.xacro config/dual_arm_positioncontrol.yaml config/dual_arm_effortcontrol.yaml
git commit -m "$(cat <<'EOF'
feat: 손목 yaw 관절용 트랜스미션 및 컨트롤러(joint12/13) 추가

배경/문제:
Task 1에서 추가한 L_wrist_yaw_joint/R_wrist_yaw_joint는 트랜스미션과
ros_control 컨트롤러 설정이 없으면 액추에이터가 없는 관절이라 Gazebo에서
전혀 움직일 수 없다.

검토한 대안과 선택 이유:
기존 joint1~11 컨트롤러 이름/PID를 재사용하는 대신 joint12/13으로 끝에
추가 - 기존 컨트롤러를 재번호 매기면 이 이름들을 참조하는 다른 launch
파일/설정이 깨질 위험이 있어 surgical하게 append만 함. PID는 jys의
Kp=40/Kd=8(다른 관절 대비 훨씬 무른 값, 컴플라이언스 목적)을 그대로
가져옴 - Task 3의 C++ Kp[DoF]/Kd[DoF] 배열과 반드시 동일하게 유지해야 함.

구현 내용:
xacro에 L_wrist_yaw_joint/R_wrist_yaw_joint용 Position+Effort
SimpleTransmission 2쌍 추가(mechanicalReduction=1, 기존 팔 관절과 동일
패턴). 두 yaml 설정 파일 각각에 joint12/13_..._controller 추가.

결과/검증:
python yaml.safe_load로 두 설정 파일 다 14개 컨트롤러 키(joint_state +
joint1~13)를 갖고 있음을 확인. Gazebo 실제 스폰 검증은 Task 3(IK가 이
관절을 인식하도록 C++ 코드 확장) 이후로 미룸 - 지금은 URDF/설정만 있고
그걸 실제로 구동할 C++ 코드가 아직 DoF=11인 상태.
EOF
)"
```

---

## Task 3: Extend `dual_arm_function.h`/`.cpp` to DoF=13, retarget IK to the wrist frame

**Files:**
- Modify: `src/dual_arm_function.h`
- Modify: `src/dual_arm_function.cpp` (`SolveIK_Position`)

**Interfaces:**
- Consumes: DoF order confirmed in Task 1 Step 5.
- Produces: `DoF = 13` everywhere; `SolveIK_Position(model, data, l_EE, r_EE, target_L, target_R, q_seed, q_out)` keeps its exact signature (callers in `main.cpp` are unaffected), but internally solves an 11-column-wide task (Waist + both arms including wrist, wrist columns zeroed) instead of 9.

- [ ] **Step 1: Update `DoF`, the order comment, and every `[DoF]`-sized array**

In `src/dual_arm_function.h`, change:

```cpp
const int DoF = 11;
// DoF 배열 순서(Pinocchio model.nq 순서와 동일해야 함, urdf 트리 순회 결과로 실측 확인됨):
// 0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:R_sp 8:R_sr 9:R_sy 10:R_e
```

to:

```cpp
const int DoF = 13;
// DoF 배열 순서(Pinocchio model.nq 순서와 동일해야 함, urdf 트리 순회 결과로 실측 확인됨,
// Task 1 Step 5에서 pinocchio 로드로 재검증):
// 0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:L_wy
// 8:R_sp 9:R_sr 10:R_sy 11:R_e 12:R_wy
```

Then find `Kp[DoF]`/`Kd[DoF]` and widen from 11 to 13 entries, inserting the soft wrist gains at indices 7 and 12 (matching Task 2's yaml `p:40/d:8`):

```cpp
double Kp[DoF] = { 1000, 500, 500, 500, 500, 500, 500, 40, 500, 500, 500, 500, 40 };
double Kd[DoF] = { 50,   30,  30,  1,   1,   1,   1,   8,  1,   1,   1,   1,   8  };
```

All other `[DoF]`-sized declarations in this file (`PD_torque`, `PD_acc`, `gravity_torque`, `nonlinear_torque`, `dynamic_torque`, `target_torque`, `dual_arm_jointp`, `dual_arm_jointv`, `dual_arm_jointp_vec`, `dual_arm_jointv_vec`, `dual_arm_jointv_lpf`, `dual_arm_jointv_lpf_vec`, `dual_arm_jointv_before`, `dual_arm_initp`, `dual_arm_commandp`, `dual_arm_targetp`, `dual_arm_targetv`, `dual_arm_targetp_vec`, `dual_arm_targeta_vec`, `q_ik_seed`, `q_ik_result`, `realtime_lift_hold_q`, and any `MatrixXd ...= MatrixXd::Zero(1,DoF)` trajectory buffers) need **no code change** — they're already sized off the `DoF` constant, so bumping `DoF` to 13 grows them automatically. Confirm this by:

```bash
grep -n '\[DoF\]\|Zero(.*DoF)\|Zero(DoF' src/dual_arm_function.h
```

and checking every hit is either `Kp`/`Kd` (already fixed above) or a bare `DoF`-driven declaration (no `{...}` literal list needing manual widening).

- [ ] **Step 2: Widen `SolveIK_Position`'s Jacobian from 9 to 11 active columns, zero the wrist columns**

Current code (`src/dual_arm_function.cpp`) builds a `MatrixXd J(6, model.nv)` where `model.nv` is now 13 (all of Waist+Head+arms), with `q_pref`/`postureErr` covering all `DoF` entries and Head explicitly masked to 0. Head already gets masked out of the null-space posture pull (`postureErr(1) = 0.0; postureErr(2) = 0.0;`) — do the same for the wrist columns, but **also zero their columns in `J` itself** (not just the null-space posture term) so the task-space solve never uses them, matching `jys`'s explicit design intent ("Wrist yaw is reserved for passive/contact alignment and must not compensate for arm pose error").

Locate:

```cpp
MatrixXd J(6, model.nv);
J.topRows<3>()    = JL.topRows<3>();   // 위치 3행만
J.bottomRows<3>() = JR.topRows<3>();
```

Add immediately after:

```cpp
// 손목 yaw(인덱스 7, 12)는 접촉 후 컴플라이언스 전용 축이다 - IK가 팔 자세
// 오차를 메우는 데 이 축을 쓰면 안 되므로 태스크 자코비안에서 완전히 제외한다.
J.col(7).setZero();
J.col(12).setZero();
```

And extend `q_pref`'s masking (currently only Head):

```cpp
VectorXd q_pref = VectorXd::Zero(DoF);
q_pref(3) = 0.80;  q_pref(4) =  0.55; q_pref(5) = -0.20; q_pref(6)  = -0.45;  // L arm
q_pref(7) = 0.80;  q_pref(8) = -0.55; q_pref(9) =  0.20; q_pref(10) = -0.45;  // R arm
```

becomes (indices shift because L_wy/R_wy are now interleaved into the arm blocks):

```cpp
VectorXd q_pref = VectorXd::Zero(DoF);
q_pref(3) = 0.80;  q_pref(4) =  0.55; q_pref(5) = -0.20; q_pref(6)  = -0.45;  // L arm (7=L_wy handled below)
q_pref(8) = 0.80;  q_pref(9) = -0.55; q_pref(10) =  0.20; q_pref(11) = -0.45; // R arm (12=R_wy handled below)
```

and after the existing `postureErr(1) = 0.0; postureErr(2) = 0.0;` (Head mask), add:

```cpp
postureErr(7)  = 0.0;  // L_wrist_yaw - no posture pull, pure contact compliance
postureErr(12) = 0.0;  // R_wrist_yaw
```

- [ ] **Step 3: Offline-verify the modified IK still converges on the existing (non-wrist) task**

Reuse the exact `waist_sweep.py` harness from earlier in this session (it already builds a pinocchio model from `urdf/dual_arm.urdf` and replicates `SolveIK_Position`) as a template — rebuild it against the *new* 13-DoF URDF, call `solve_ik` for `L_wrist_ik_joint`/`R_wrist_ik_joint` frames (not `L_EE`/`R_EE`!) with the same `x=0.45, z=1.2` test point used before, and confirm:
1. `q[7]` and `q[12]` (the two wrist entries) stay at exactly `0.0` after convergence (proof the zeroed Jacobian columns really keep IK from touching them).
2. The residual error and other joint angles are in the same ballpark as the pre-wrist sweep table produced earlier in this session (waist ≈0-38° range for y=0-0.48) — a large regression here means the column-zeroing or index remap has a bug.

```bash
python3 /tmp/claude-1000/-home-jungmin/35a45825-9dca-4d51-b3e9-3c9a3b3dcbb2/scratchpad/waist_sweep_wrist_check.py
```

(Write this as a copy of the earlier `waist_sweep.py` with `DoF`/`Kp`/`Kd`/column-zeroing updated to match Steps 1-2, and `l_EE`/`r_EE` swapped to `model.getFrameId('L_wrist_ik_joint')`/`model.getFrameId('R_wrist_ik_joint')`.)

- [ ] **Step 4: Commit**

```bash
git add src/dual_arm_function.h src/dual_arm_function.cpp
git commit -m "$(cat <<'EOF'
feat: DoF 11->13 확장, SolveIK_Position이 손목 프레임을 타겟하고 손목 컬럼은 0으로 고정

배경/문제:
Task 1~2에서 URDF/컨트롤러에 손목 관절을 추가했지만, IK를 포함한 C++ 제어
코드는 여전히 DoF=11 기준이라 이 관절의 존재를 전혀 모른다. jys 브랜치는
이 문제를 팔별로 IK를 완전히 분리해서 풀어(Waist 미포함) 해결했는데, 그건
이 브랜치가 DEFENSE_GUIDE.md에서 이미 해결한 "허리 충돌" 문제(스택
자코비안으로 왼팔/오른팔이 같은 waist 관절에 상반된 명령을 내는 것을
방지)를 되돌리는 것이라 그대로 포팅하지 않기로 함.

검토한 대안과 선택 이유:
jys의 손목 컬럼 제로잉 아이디어(IK가 손목을 절대 안 쓰게 강제 - "Wrist yaw
is reserved for passive/contact alignment")는 그대로 채택. 대신 이걸
jys처럼 팔별 분리 자코비안이 아니라, 이 브랜치의 기존 스택 6x9 자코비안을
6x11로 넓히고 손목 2개 컬럼(인덱스 7, 12)만 0으로 고정하는 방식으로
구현 - Waist 공유 구조는 그대로 유지하면서 손목만 완전히 배제.

구현 내용:
DoF=11->13, Kp/Kd 배열에 손목 인덱스(7,12) 소프트 게인(40/8, config yaml과
동일값) 추가. SolveIK_Position: J.col(7)/J.col(12)를 setZero(), q_pref/
postureErr도 손목 인덱스는 0으로 마스킹(Head와 동일한 패턴). 나머지
[DoF] 배열들은 DoF 상수 기반 자동 크기조정이라 코드 변경 불필요함을 grep로
확인.

결과/검증:
waist_sweep.py를 13-DoF URDF + L_wrist_ik_joint/R_wrist_ik_joint 프레임
기준으로 재작성해 오프라인 실행: 수렴 후 q[7]/q[12]가 정확히 0.0으로
남아있음(손목 컬럼 제로잉이 실제로 작동), 나머지 관절각/잔차오차가 이전
9-column 스윕 결과와 같은 범위임을 확인(회귀 없음). Gazebo 실행 검증은
Task 4(main.cpp 통합) 이후로 미룸 - 지금은 main.cpp가 여전히 옛 DoF=11
가정으로 배열을 접근해 컴파일이 깨진 상태.
EOF
)"
```

---

## Task 4: Wire the IK-frame remap + wrist compliance state machine into `main.cpp`

**Files:**
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: `L_wrist_ik_joint`/`R_wrist_ik_joint`/`L_grip_joint`/`R_grip_joint` frame names (Task 1); `DoF=13`, wrist indices 7/12 (Task 3).
- Produces: `dual_armjoint12_pub`/`dual_armjoint13_pub` publishers wired to `dual_arm_targetp[7]`/`target_torque[7]` and `[12]`; a `mapNominalTargetsToIkTargets` lambda used everywhere `SolveIK_Position` is currently called.

- [ ] **Step 1: Add the two new frame ids next to the existing `l_EE`/`r_EE` lookup**

Find where `l_EE`/`r_EE` (frame indices for `L_EE_joint`/`R_EE_joint`) are currently obtained via `model.getFrameId(...)`. Add alongside:

```cpp
pinocchio::FrameIndex l_ik_EE = model.getFrameId("L_wrist_ik_joint");
pinocchio::FrameIndex r_ik_EE = model.getFrameId("R_wrist_ik_joint");
pinocchio::FrameIndex l_grip_EE = model.getFrameId("L_grip_joint");
pinocchio::FrameIndex r_grip_EE = model.getFrameId("R_grip_joint");
```

Keep the existing `l_EE`/`r_EE` (→ `L_EE_joint`/`R_EE_joint`) exactly as-is — all of the existing `objL/R`, `transportL/R`, `standoffL/R` geometry in `buildGraspPipelineFromDetection` keeps targeting this nominal frame unchanged.

- [ ] **Step 2: Add the offset-remap lambda**

Immediately after the frame-id block from Step 1:

```cpp
// 기존 objL/R, transportL/R 등은 전부 L_EE_joint/R_EE_joint(nominal) 기준으로
// 계산된다 - 이 수학은 그대로 둔다. IK를 실제로 손목 IK 프레임(L_wrist_ik_joint)에
// 대고 풀어야 하므로, 호출 직전에 이 오프셋만큼 목표를 보정한다. 오프셋은 seed
// 자세(손목 yaw=0, 아직 컴플라이언스로 안 밀린 상태) 기준 1회 계산 - 손목은
// SolveIK_Position에서 컬럼이 0으로 고정돼 궤적 재생 중 안 움직이므로 이 오프셋은
// 궤적 내내 상수다.
auto mapNominalTargetsToIkTargets = [&](const VectorXd& q_ref,
                                        const Vector3d& nominal_target_L, const Vector3d& nominal_target_R,
                                        Vector3d& ik_target_L, Vector3d& ik_target_R) {
    pinocchio::forwardKinematics(model, data, q_ref);
    pinocchio::updateFramePlacements(model, data);
    const Vector3d left_nominal_offset  = data.oMf[l_EE].translation() - data.oMf[l_ik_EE].translation();
    const Vector3d right_nominal_offset = data.oMf[r_EE].translation() - data.oMf[r_ik_EE].translation();
    ik_target_L = nominal_target_L - left_nominal_offset;
    ik_target_R = nominal_target_R - right_nominal_offset;
};
```

- [ ] **Step 3: Insert the remap at every existing `SolveIK_Position` call site**

```bash
grep -n "SolveIK_Position(" src/main.cpp
```

For each call, the two `Vector3d` position arguments currently passed straight from `objL/objR`/`transportL/transportR`/`standoffL/standoffR` etc. must instead go through `mapNominalTargetsToIkTargets` first, using the seed pose already available at that call site (`base_seed`/`q_seed`/whatever local variable holds the current joint-angle seed for that call) as `q_ref`. Example for the existing pattern:

```cpp
// before:
dualarm.SolveIK_Position(model, data, l_EE, r_EE, standoffL, standoffR, base_seed, q_out);

// after:
Vector3d ikL, ikR;
mapNominalTargetsToIkTargets(base_seed, standoffL, standoffR, ikL, ikR);
dualarm.SolveIK_Position(model, data, l_ik_EE, r_ik_EE, ikL, ikR, base_seed, q_out);
```

Apply this transform at **every** call site found by the grep (there are multiple — one per trajectory segment build, matching the pattern jys used at 4 call sites). Do not skip any — a missed one will silently solve IK for the old `l_EE`/`r_EE` frame instead of the wrist-IK frame, producing a hand position off by the `L_wrist_ik_joint`↔`L_EE_joint` offset (roughly 5cm, per Task 1's `-0.10` vs `-0.15` origins).

- [ ] **Step 4: Add the wrist-alignment + grasp-contact state machine**

Find the F/T sensor read + low-pass-filter block (`left_ft_force_lpf`, `right_ft_force_lpf`, and by this point in the plan also `left_ft_torque_lpf`/`right_ft_torque_lpf` if not already present — check with `grep -n "left_ft_torque" src/main.cpp`; if torque isn't currently being read/filtered at all, add it following the exact same pattern as the existing force read/LPF code, reading `msg->wrench.torque.x/y/z` instead of `.force.x/y/z`).

Add these new constants near the other admittance tuning constants in `dual_arm_function.h` (values copied from `jys`, already validated there against the same 10N squeeze target this branch also uses):

```cpp
const double WRIST_ALIGN_CONTACT_FORCE     = 2.0;   // [N] below this, don't treat zero torque as "aligned"
const double WRIST_ALIGN_TORQUE_THRESHOLD  = 0.15;  // [N*m] allowed residual moment once face-aligned
const int    WRIST_ALIGN_HOLD_TICKS        = 200;   // 0.20s @ 1kHz - must stay settled this long
```

And these new state variables (alongside the other grasp-phase state already in `dual_arm_function.h`):

```cpp
bool wrist_alignment_ready = false;
int  wrist_alignment_ticks = 0;
```

In `main.cpp`'s per-tick admittance block, right after the F/T LPF update, add the gating check (this only *reads* torque to decide readiness — it does **not** command the wrist joint directly; the wrist is driven passively by its soft `Kp=40` PD tracking whatever target trajectory value it already has, which stays 0 since `SolveIK_Position` never moves it):

```cpp
if (!wrist_alignment_ready) {
    const bool alignment_contact =
        compressive_force_L >= WRIST_ALIGN_CONTACT_FORCE &&
        compressive_force_R >= WRIST_ALIGN_CONTACT_FORCE;
    const bool alignment_torque_ok =
        std::abs(left_ft_torque_lpf(0))  <= WRIST_ALIGN_TORQUE_THRESHOLD &&
        std::abs(left_ft_torque_lpf(2))  <= WRIST_ALIGN_TORQUE_THRESHOLD &&
        std::abs(right_ft_torque_lpf(0)) <= WRIST_ALIGN_TORQUE_THRESHOLD &&
        std::abs(right_ft_torque_lpf(2)) <= WRIST_ALIGN_TORQUE_THRESHOLD;
    if (alignment_contact && alignment_torque_ok) {
        wrist_alignment_ticks++;
        if (wrist_alignment_ticks >= WRIST_ALIGN_HOLD_TICKS) {
            wrist_alignment_ready = true;
            ROS_INFO("Wrist face alignment ready: qL=%.3f qR=%.3f rad", dual_arm_jointp[7], dual_arm_jointp[12]);
        }
    } else {
        wrist_alignment_ticks = 0;
    }
}
```

`compressive_force_L`/`compressive_force_R` already exist in this branch's admittance code (the y-projected squeeze force feeding the K=0 integrator) — reuse them as-is, do not recompute via `jys`'s dynamic left-to-right axis (that's an explicitly out-of-scope generalization for this port, see plan header).

Gate the **existing** squeeze-force ramp / transport-phase advance on `wrist_alignment_ready` the same way this branch's current phase-advance logic already gates on other readiness flags — find the existing phase-transition condition (from contact/squeeze phase to lift/transport phase) and add `&& wrist_alignment_ready` to it, so transport does not begin until the wrist has passively settled.

- [ ] **Step 5: Add the 2 new joint publishers and wire the DoF-indexed values through**

Find the existing `dual_armjoint1_pub`...`dual_armjoint11_pub` declarations (both position-mode and effort-mode `#if`/`#else` blocks) and add, in each block:

```cpp
ros::Publisher dual_armjoint12_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint12_position_controller/command", 100);
ros::Publisher dual_armjoint13_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint13_position_controller/command", 100);
```

(and the `_effort_controller` variant in the effort-mode block). Declare two new message variables alongside the existing `shoulder_pitch_l_joint_msg` etc.:

```cpp
std_msgs::Float64 wrist_yaw_l_joint_msg, wrist_yaw_r_joint_msg;
```

In the existing publish-assembly block:

```cpp
shoulder_pitch_l_joint_msg.data = dual_arm_targetp[3];
...
elbow_l_joint_msg.data          = dual_arm_targetp[6];
wrist_yaw_l_joint_msg.data      = dual_arm_targetp[7];    // new
shoulder_pitch_r_joint_msg.data = dual_arm_targetp[8];    // index shifts: was [7]
shoulder_roll_r_joint_msg.data  = dual_arm_targetp[9];    // was [8]
shoulder_yaw_r_joint_msg.data   = dual_arm_targetp[10];   // was [9]
elbow_r_joint_msg.data          = dual_arm_targetp[11];   // was [10]
wrist_yaw_r_joint_msg.data      = dual_arm_targetp[12];   // new
```

**This index shift is the highest-risk edit in this task** — every `dual_arm_targetp[i]`/`target_torque[i]` read at index ≥7 in the existing code (both the position-mode and effort-mode assembly blocks, and anywhere else `[7]` through `[10]` is read as "R arm") must shift by +1 to account for `L_wy` being inserted at 7. Search exhaustively:

```bash
grep -n 'targetp\[7\]\|targetp\[8\]\|targetp\[9\]\|targetp\[10\]\|target_torque\[7\]\|target_torque\[8\]\|target_torque\[9\]\|target_torque\[10\]' src/main.cpp
```

Fix every hit, then re-run the same grep for `\[11\]`/`\[12\]` to confirm the new R-arm/R-wrist indices are used consistently.

Finally add the two new `.publish(...)` calls next to the existing 11:

```cpp
dual_armjoint12_pub.publish(wrist_yaw_l_joint_msg);
dual_armjoint13_pub.publish(wrist_yaw_r_joint_msg);
```

- [ ] **Step 6: Build**

```bash
cd /home/jungmin/friend_ws_88pct
catkin_make -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -60
```

Expected: clean build. Any compile error at this stage almost certainly means Step 5's index shift was missed somewhere — grep again before debugging further.

- [ ] **Step 7: Commit**

```bash
cd /home/jungmin/friend_ws_88pct/src/Dual_Arm_ROS_Simulation
git add src/main.cpp src/dual_arm_function.h
git commit -m "$(cat <<'EOF'
feat: main.cpp에 손목 IK 프레임 리매핑 + 접촉기반 정렬 상태머신 연결

배경/문제:
Task 3까지 IK/URDF/컨트롤러는 손목을 알지만, main.cpp는 여전히 옛
L_EE_joint/R_EE_joint를 IK 타겟으로 쓰고 DoF=11 기준 인덱스로 배열에
접근해 컴파일이 깨진 상태였다.

검토한 대안과 선택 이유:
jys의 "practical mapping rule"(design doc에 기술, 실제 구현도 동일하게
확인됨)을 그대로 채택 - 기존 grasp_offset 기반 objL/R, transportL/R 계산은
전혀 안 건드리고, SolveIK_Position 호출 직전에 nominal(L_EE_joint) ->
IK(L_wrist_ik_joint) 오프셋만 빼주는 람다 하나로 리매핑. 손목 정렬은 jys처럼
액티브 각도 계산이 아니라 접촉력+토크 임계값 기반 게이팅으로 "언제 다음
단계로 넘어갈지"만 판단(실제 정렬은 Task 3의 소프트 Kp=40 PD가 물리적으로
수행) - 이 branch의 기존 admittance 자체는 안 바꾸고 게이트 조건만 추가.

구현 내용:
l_ik_EE/r_ik_EE/l_grip_EE/r_grip_EE 프레임id 추가. mapNominalTargetsToIkTargets
람다 추가, 모든 SolveIK_Position 호출부에 적용. WRIST_ALIGN_* 상수와
wrist_alignment_ready/ticks 상태 추가, F/T 토크 LPF 기반 게이팅 로직을
admittance 루프에 삽입해 기존 phase 전환 조건에 && wrist_alignment_ready
추가. joint12/13 퍼블리셔 추가 및 DoF 인덱스 시프트(R팔 관련 인덱스 전부
+1) 전체 반영.

결과/검증:
catkin_make 클린 빌드 확인. grep으로 targetp[7]~[10]/target_torque[7]~[10]
잔여 참조가 없음(전부 새 인덱스로 이동)을 확인. Gazebo 실행 검증은 다음
태스크로 미룸.
EOF
)"
```

---

## Task 5: Gazebo end-to-end verification + palm mount re-tuning

**Files:**
- No new source changes expected — this task is verification + iterative constant tuning in files already touched (`urdf/dual_arm.xacro` palm collision origin, `dual_arm_function.h` `WRIST_ALIGN_*`/`Kp`/`Kd` wrist entries).

- [ ] **Step 1: Clean-restart Gazebo**

```bash
ps aux | grep -E "ros|gz|gazebo" | grep -v grep | awk '{print $2}' | xargs -r kill -9
cd /home/jungmin/friend_ws_88pct && source devel/setup.bash
roscore &
roslaunch dual_arm gazebo.launch &
roslaunch dual_arm aruco_detection.launch &
rosrun dual_arm dual_arm_command
```

- [ ] **Step 2: Run a vision pick to a moderate target (not the demo-extreme y=0.25 from earlier) and observe**

Use `0.45 0.15 1.2` (the previously-validated baseline target) first, not the wider waist-swing target, to isolate wrist-related issues from the separate waist-range tuning already done. Watch for:
1. Does the arm reach standoff/contact without the wrist visibly flopping/oscillating uncontrollably (a too-soft `Kp`/too-loose `WRIST_ALIGN_TORQUE_THRESHOLD` symptom)?
2. Does `wrist_alignment_ready` ever become true (check `rostopic echo` on wherever this gets logged, or the `ROS_INFO` line added in Task 4 Step 4 — `rosservice`/terminal output)? If it never fires, the palm mount origin (`-0.0375 -0.005` from Task 1) likely doesn't actually contact the box face at all with this branch's arm geometry (unlike jys's, which has different link lengths) — this needs re-measurement, same as the original disc pad needed 3 rounds of offline recomputation.
3. Does the box stay gripped through lift/transport without the rotate-and-fall failure mode this whole feature exists to fix?

- [ ] **Step 3: If contact never registers, re-derive the palm collision origin offline (same method as the original disc pad)**

```bash
python3 - <<'EOF'
import pinocchio as pin
import numpy as np
model = pin.buildModelFromUrdf('/home/jungmin/friend_ws_88pct/src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf')
data = model.createData()
l_wrist = model.getFrameId('L_wrist_yaw_joint')
l_ee = model.getFrameId('L_EE_joint')
# Replay a representative grasp-configuration q (copy the actual q_ik_result
# logged from the Step 2 Gazebo run at the moment of closest approach,
# or reuse the neutral q_pref-seeded pose from waist_sweep.py) and inspect
# where L_EE's local -Y axis actually points relative to the box's known
# world-Y face normal, the same way fk_pad_calc.cpp did for the old disc pad.
EOF
```

Adjust the `L_EE_palm_collision`/`L_EE_grip_pad_visual` (and mirrored R) `origin xyz` in `urdf/dual_arm.xacro` based on this, re-run `xacro` (Task 1 Step 4), rebuild, and re-test. Expect 1-3 iterations, matching the original pad's tuning history.

- [ ] **Step 4: Tune `WRIST_ALIGN_TORQUE_THRESHOLD`/`WRIST_ALIGN_HOLD_TICKS` if alignment is flagged ready too early (still visibly tilted) or never**

Too early → tighten (lower) `WRIST_ALIGN_TORQUE_THRESHOLD`. Never triggers → loosen it, or check `Kp[7]`/`Kp[12]` isn't so soft the wrist never settles within `WRIST_ALIGN_HOLD_TICKS` — raise `Kd[7]`/`Kd[12]` (damping) before touching `Kp` (stiffness), to avoid re-fighting the whole point of the compliance design.

- [ ] **Step 5: Full pick-lift-transport-place run, then commit final tuning**

```bash
git add urdf/dual_arm.xacro src/dual_arm_function.h
git commit -m "$(cat <<'EOF'
tune: 손목 컴플라이언스 파지 Gazebo 실측 튜닝(패드 마운트/정렬 임계값)

배경/문제:
Task 1의 팔 손바닥 콜리전 원점(-0.0375/-0.005)과 Task 4의 WRIST_ALIGN_*
임계값은 jys 브랜치 수치를 그대로 가져온 추정값이었다 - 이 브랜치의 실제
팔 링크 길이/관절 배치가 달라 그대로 맞을 리 없고, 옛 원판 패드도 실측
3회 반복 끝에 확정됐던 전례가 있다.

검토한 대안과 선택 이유:
[Gazebo 실측 결과에 따라 채운다 - 실제 반복 횟수, 어떤 값을 얼마나
움직였는지, 최종 확정값과 그 근거]

구현 내용:
[실제 반영된 origin/threshold 최종값]

결과/검증:
[전체 pick-lift-transport-place 1회 이상 성공 로그, 손목이 접촉 후 육안상
박스 면에 평행하게 정렬되는지 확인, 그립 놓침/회전 낙하 재발 여부]
EOF
)"
```

(This final commit message has bracketed placeholders because its content depends on Gazebo results this plan cannot predict — fill them in from the actual run before committing; do not commit with the brackets still in place.)

---

## Self-Review Notes

- **Spec coverage**: Task 1 covers the URDF/mechanical redesign (root-cause fix for "패드가 완벽히 평행하지 않아"). Task 2 covers the ros_control wiring the new joints need to move at all. Task 3 covers IK not regressing the waist-sharing fix while giving the wrist a reserved, unused-by-IK column. Task 4 covers the geometry remap (keeping all existing `grasp_offset`/box-size work untouched) plus the passive-compliance gating mechanism. Task 5 covers the empirical re-tuning every prior grasp-geometry change in this repo's history has needed.
- **Explicitly out of scope** (flag to user before starting, don't silently add): jys's dynamic left-to-right squeeze axis (this branch keeps its existing fixed-world-Y squeeze assumption); jys's `grasp_contact_ready`/`grasp_acquired_once`/`realtime_lift_*` broader state-machine rework (only the narrower `wrist_alignment_ready` gate is ported); any change to `command_mode 1`/`2` (joint-space/Cartesian single-shot sim modes) beyond what Task 4 Step 3's exhaustive grep turns up.
- **Placeholder scan**: Task 5 Step 5's commit message is the one intentionally-incomplete block in this plan, called out explicitly as depending on empirical results — every other step has concrete code/commands.
