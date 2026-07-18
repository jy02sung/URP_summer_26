# Box Lifting Missions

## 목표

`admittance-control`의 검증된 양팔 제어를 유지하면서 `jys`에서 개발한 좌우 wrist yaw를 선별 이식한다.
최종 동작은 물체를 제자리에서 파지하고 수직으로 들어 올린 뒤, 같은 위치에 다시 내려놓는 것이다.

```text
SCAN → APPROACH → WRIST_ALIGN → SQUEEZE → LIFT → HOLD → LOWER → RELEASE → RETURN
```

## 작업 원칙

- 작업 저장소: `/home/jys/catkin_ws_jmp/src/urp_summer_26`
- 작업 브랜치: `box-lifting`
- donor 저장소: `/home/jys/catkin_ws`의 `jys` 브랜치
- 기반 제어는 `jmp`의 Cartesian admittance를 유지한다.
- `jys` 전체 제어 코드를 한꺼번에 복사하지 않는다.
- 각 미션은 별도 커밋으로 남긴다.
- 각 미션 통과 전에는 다음 미션을 시작하지 않는다.
- 코드 변경 후 workspace root에서 `catkin_make -DCMAKE_BUILD_TYPE=Release`를 실행한다.
- ROS 시험 전 `source devel/setup.bash`를 실행한다.
- Gazebo 시험 전 기존 ROS/Gazebo 프로세스를 확인한다.
- GUI에서 진동, 자세 드리프트, 비의도 관절 운동을 확인한다.
- 시험 후 Codex가 실행한 프로세스만 종료한다.

## 완료 기준

- [x] 양쪽 wrist yaw/pitch가 포함된 15DoF 모델이 안정적으로 스폰된다.
- [x] waist와 head를 사용하지 않는 arm-only IK가 유지된다.
- [ ] 손목이 접촉면에 제한적으로 정렬되고 관절 끝으로 열리지 않는다.
- [ ] 양손이 목표 압착력을 안정적으로 유지한다.
- [ ] 물체가 제자리에서 수직 상승한다.
- [ ] 물체를 일정 시간 유지한다.
- [ ] 물체가 원래 위치에 다시 놓인다.
- [ ] 손을 안전하게 놓고 팔이 복귀한다.

---

## Mission 0 — 기준 동작 고정

### 목표

수정 전 `box-lifting` 기준 동작을 재현하고 비교용 데이터를 남긴다.

### 작업

- 현재 11DoF 모델을 Release로 빌드한다.
- Gazebo GUI와 ArUco detector를 실행한다.
- README의 vision 명령으로 기존 pick-and-place를 한 번 실행한다.
- 물체 시작 위치, 검출 위치, 최종 위치를 기록한다.
- 양팔 joint state와 F/T sensor 토픽이 정상인지 확인한다.

### 통과 조건

- [x] 빌드 성공
- [x] 11개 controller 로드 성공
- [x] `/dual_arm/joint_states`에 NaN 없음
- [x] 카메라 약 20 Hz 수신
- [x] ArUco marker 검출 성공
- [x] 양팔이 물체를 실제로 이동시킴

### 결과 기록

```text
Date: 2026-07-17
Base commit: 32bf5a6
Object start pose:  [0.4500, -0.1500, 1.2000], yaw 0 deg
Detected box center: [0.4690, -0.1720, 1.1910]
Transport target:   [0.4500,  0.1700, 1.2000]
Object final pose:  [0.4404,  0.1548, 1.2000], yaw about 4.42 deg
Camera rate: about 20.1 Hz
Controllers: joint_state_controller + 11 effort controllers, all running
Known issues:
- Detected center differs from initial truth by about [+1.9, -2.2, -0.9] cm.
- Final target error is about [-1.0, -1.5, 0.0] cm with about 4.42 deg yaw.
- TrajectoryDone was not published during an additional 70 s observation.
- Arms remained in a non-zero holding pose after the object stopped.
- The branch does not contain the aruco_ros executable; the jys build was used temporarily.
- Deleting a nonexistent place_indicator emits a harmless Gazebo error.
```

---

## Mission 1 — Wrist URDF 이식

### 목표

현재 손바닥 형상을 유지하면서 좌우 wrist yaw link/joint만 추가한다.

### 작업

- `jys`의 wrist yaw link, joint, inertia, damping, limit를 참고한다.
- `L_EE_joint`와 `R_EE_joint`를 각 wrist link 아래로 재배치한다.
- F/T sensor joint 구조를 유지한다.
- 새 손바닥 패드 중심에 맞춰 contact/IK frame 위치를 정의한다.
- transmission과 Gazebo joint 설정을 추가한다.
- xacro에서 URDF를 다시 생성하고 parser로 검사한다.

### 주의

- `jys`의 손바닥 collision과 grip pad 형상은 복사하지 않는다.
- wrist pitch는 추가하지 않는다.
- URDF 물리 한계 `±0.9 rad`는 정상 운전 범위가 아니다.

### 통과 조건

- [x] xacro 변환 성공
- [x] URDF parser 성공
- [x] 좌우 wrist yaw joint 존재
- [x] EE와 F/T sensor가 wrist 아래에 연결됨
- [x] 기존 손바닥 visual/collision 유지

### 결과 기록

```text
Date: 2026-07-17
Wrist chain: elbow -> wrist_yaw(revolute) -> EE(fixed)
Arm reach: 0.15 m + 0.15 m = 기존 0.30 m 유지
Wrist limit: ±0.9 rad, effort 12 Nm, velocity 4 rad/s, damping 1
Frames: L/R_wrist_ik_frame 및 기존 grip disc 중심의 L/R_grip_frame 추가
F/T: 기존 L/R_EE_joint 이름과 Gazebo sensor plugin 유지
Palm: 기존 jmp EE mesh 및 주황색 원형 visual/collision 유지
Transmissions: 13개 joint 순서로 확장하되 controller/제어 코드는 아직 변경하지 않음
Validation: xacro 성공, check_urdf 성공, Release catkin_make 성공
Runtime spawn test: Mission 2에서 dual_arm_main 없이 수행 예정
```

---

## Mission 2 — Spawn-only 물리 안정성

### 목표

제어 노드 없이 wrist가 추가된 모델 자체의 물리 안정성을 검증한다.

### 작업

- `dual_arm_main` 없이 스폰하는 진단 launch를 만든다.
- 가능하면 physics paused 상태에서 먼저 스폰한다.
- unpause 후 최초로 움직이는 joint/link를 확인한다.
- `/gazebo/model_states`와 `/gazebo/get_joint_properties`를 확인한다.

### 통과 조건

- [x] model pose/twist에 NaN 없음
- [x] wrist가 joint limit로 튀지 않음
- [x] elbow와 shoulder가 비정상적으로 이동하지 않음
- [x] 모델이 10초 이상 안정적으로 유지됨

### 결과 기록

```text
Date: 2026-07-17
Launch: gazebo_spawn_only.launch, paused:=true, gui:=true
Control isolation: controller spawner와 dual_arm_main을 실행하지 않음
Paused pose: 양쪽 shoulder/elbow/wrist 모두 0 rad
Observed simulation time: 약 102 s
Finite check: model pose/twist와 측정한 모든 arm joint 값이 finite
Wrist after 약 92 s: left 0.00035 rad, right 0.00063 rad
Maximum passive arm displacement: 약 0.089 rad
10 s wrist drift: left +0.000040 rad, right +0.000077 rad
10 s maximum joint drift: shoulder yaw 약 0.0029 rad, 좌우 대칭
Model twist: linear 약 1.8e-7 m/s 이하, angular 약 5.9e-7 rad/s 이하
Conclusion: 손목 삽입으로 인한 NaN, joint-limit 점프, 비대칭 startup 폭주는 재현되지 않음.
Note: 무토크 자유 관절이므로 중력에 의한 약 0.09 rad의 대칭적 passive settling은 존재함.
```

### 실패 시

제어 코드를 수정하지 말고 wrist inertia, collision, fixed-joint chain부터 조사한다.

---

## Mission 2.5 — 손바닥 접촉 구조 일반화

### 목표

특정 박스 위치와 특정 IK 자세를 역산해 만든 패드 각도를 제거하고, 손목 순응으로 정렬 가능한
좌우 대칭의 중립 손바닥을 만든다.

### 작업

- 패드 법선을 EE 로컬 Y축에 정확히 정렬한다.
- 좌우 패드 위치와 자세를 완전히 대칭으로 만든다.
- `grip_frame`을 각 패드 중심과 동일한 자세로 갱신한다.
- 기존 EE mesh, 패드 크기, 마찰 및 F/T sensor 구조를 유지한다.
- spawn-only GUI에서 self-collision, NaN 및 startup drift를 다시 검사한다.

### 통과 조건

- [x] 좌우 패드 transform이 기하학적으로 대칭
- [x] xacro 및 URDF parser 성공
- [x] 기존 EE/F/T 구조 유지
- [x] spawn-only 상태에서 NaN과 손목 joint-limit 점프 없음
- [x] GUI에서 패드가 손바닥 양쪽의 중립 위치에 보임

### 결과 기록

```text
Date: 2026-07-17
Left pad:  xyz=[0, -0.0325, 0], rpy=[+pi/2, 0, 0], normal=-EE local Y
Right pad: xyz=[0, +0.0325, 0], rpy=[-pi/2, 0, 0], normal=+EE local Y
Removed: 특정 IK 자세와 world Y를 역산한 비대칭 pad transform
Preserved: EE mesh, disc radius/length, friction/contact 설정, EE fixed joint, F/T plugins
Validation: mirrored transform assertion, xacro, check_urdf, Release build 성공
GUI spawn-only: paused 초기 형상 확인 후 15 s unpaused 검사
Wrist after 15 s: left 0.000055 rad, right 0.000093 rad
Result: finite model state, self-collision 및 wrist joint-limit jump 없음
```

---

## Mission 2.6 — 15DoF wrist flexion URDF 확장

### 목표

양팔에 wrist flexion(기존 joint 이름은 wrist_pitch)을 추가해 손바닥이
앞뒤로 젖혀지며 물체 면과 평행하게 정렬될 수 있는 15DoF 기 구조를 만든다.

### 작업

- yaw와 손바닥 사이에 좌우 wrist pitch link/joint를 추가한다.
- 손바닥 접촉면 법선(local Y)과 수직인 local X를 회전축으로 사용한다.
- 전체 elbow-to-EE 길이 0.30 m를 유지한다.
- pitch link에는 collision geometry를 두지 않는다.
- pitch damping, limit 및 Gazebo implicit damping을 설정한다.
- controller와 제어 코드 이식 전에 spawn-only GUI 안정성을 확인한다.

### 통과 조건

- [x] 좌우 wrist pitch joint가 포함된 URDF parser 성공
- [x] elbow → yaw → pitch → EE 체인 확인
- [x] 기존 중립 패드와 F/T 구조 유지
- [x] paused 초기 wrist yaw/pitch가 모두 0 rad
- [x] unpause 후 NaN, self-collision 및 joint-limit 점프 없음
- [x] 모델이 15초 이상 안정적으로 유지됨

### 결과 기록

```text
Date: 2026-07-17
Kinematic chain: elbow -(0.15 m)-> yaw -(0.12 m)-> pitch -(0.03 m)-> EE
Flexion joint (`wrist_pitch_joint`): axis X, limit ±0.6 rad, damping 1.5, effort 12 Nm, velocity 3 rad/s
Pitch link: mass 0.15 kg, visual only, collision 없음
Model: URDF non-fixed joint 15개, check_urdf 및 Release build 성공
Paused: left/right yaw/pitch 모두 0 rad
15 s spawn-only: 모델 및 손목 상태 finite, yaw 최대 |각도| 0.000110 rad
15 s flexion: 최대 |각도| 0.000613 rad, 최대 |속도| 0.000930 rad/s
Manual direction check: 좌우 wrist_pitch_joint에 +0.45/-0.45 rad을 적용해 손바닥 앞/뒤 굽힘 확인
Result: NaN, 진동, self-collision 및 joint-limit jump 없음
```

---

## Mission 3 — 15DoF controller와 상태 매핑

### 목표

11DoF 제어 경로를 15DoF로 확장한다.

### Pinocchio 순서

```text
0  Waist
1  Head yaw
2  Head pitch
3  L shoulder pitch
4  L shoulder roll
5  L shoulder yaw
6  L elbow
7  L wrist yaw
8  L wrist pitch
9  R shoulder pitch
10 R shoulder roll
11 R shoulder yaw
12 R elbow
13 R wrist yaw
14 R wrist pitch
```

### 작업

- `DoF=15`, `ARM_DOF=6`으로 확장한다.
- controller YAML과 launch에 wrist controller 네 개를 추가한다.
- effort publisher 네 개를 추가한다.
- joint state는 이름 기반 매핑으로 읽는다.
- ROS controller 순서와 Pinocchio 순서를 명시적으로 변환한다.

### 통과 조건

- [x] 15개 controller 로드 성공
- [x] `/dual_arm/joint_states`에 wrist joint 포함
- [x] 모든 joint 값 유한
- [x] waist, arms, head가 시작 자세를 유지
- [x] wrist 0 rad hold 성공

### 결과 기록

```text
Date: 2026-07-19 (X-Z wrist redesign)
Transmission/controller order: waist, left arm 6, right arm 6, head yaw/pitch
Pinocchio order: waist, head yaw/pitch, left arm 6, right arm 6
State mapping: /dual_arm/joint_states의 배열 위치 대신 joint name 기반으로 Pinocchio index에 매핑
Runtime assertion: Pinocchio nq=15, nv=15 및 각 q index/name 출력
Controllers: joint_state_controller + effort-interface JointPositionController 15개 running
Joint states: 15개 이름 수신, position/velocity 모두 finite
Control law: 적분항 없는 PD, arm P=1.0/D=0.2, X-Z wrist P=0.5/D=0.1
Startup: Gazebo paused, 관절 15개 0 rad 설정, controller 로드 후 1배속 unpause
Rejected test: 외부 RNEA/effort 즉시 시작은 좌우 결합 진동과 관절 한계 충돌을 재현해 제외
30 s hold: 15 joint finite, 최대 |속도|=0.01435 rad/s(초기 Head yaw), 양손목 속도 <3.4e-5 rad/s
Static sag: head pitch 0.251 rad, shoulder 0.05~0.08 rad; 진동 없는 무적분 PD의 정상상태 오차
Flexion tracking: 좌우 +0.30 rad 명령에 약 +0.260 rad, -0.30 rad에 약 -0.263/-0.260 rad로 무진동 수렴
Neutral return after 8 s: L=-0.00189 rad, R=-0.00273 rad
```

---

## Mission 4 — Arm-only IK 복구

### 목표

양팔 IK가 각 팔의 6개 joint만 사용하고 waist/head를 움직이지 않게 한다.

### 작업

- Left IK columns: `3,4,5,6,7,8`
- Right IK columns: `9,10,11,12,13,14`
- waist/head는 seed 값을 그대로 유지한다.
- wrist가 추가된 새 IK frame과 contact frame을 사용한다.
- 기존 11DoF 접근 목표를 15DoF 모델에서 다시 검증한다.

### 통과 조건

- [x] head pitch 동작 중 waist/arms/head yaw 고정
- [x] arm IK 중 waist/head 고정
- [x] 양팔이 좌우 독립 위치 목표에 0.3mm 이내로 도달
- [x] wrist가 IK 계산 때문에 정상 범위 `±0.35 rad`를 벗어나지 않음

### 결과 기록

```text
Date: 2026-07-19
Solver: 좌우 팔 각 6DoF column만 사용하는 DLS position IK
Locked joints: Waist, Head yaw, Head pitch는 seed를 정확히 유지
IK frames: L/R_wrist_ik_frame
Numerical error: Left 0.233mm, Right 0.210mm
Wrist bound: 절대값 0.35rad 이내
Runtime hold: 중력 feedforward + I=0 PD, GUI 안정화 후 최대 관절속도 2.6e-5rad/s
Model correction: Pinocchio URDF의 R shoulder pitch 축을 Gazebo xacro와 같은 Y축으로 수정
Object baseline: 15cm box, center [0.45, 0, 1.225]
```

---

## Mission 5 — Wrist 제한 순응

### 목표

손목이 접촉면에 맞춰 회전하되 바깥으로 계속 열리지 않게 한다.

### 제어 정책

```text
비접촉/접근: 0 rad 중심 유지
접촉 정렬: 절대 범위 ±0.30~0.35 rad에서만 순응
정렬 완료: 현재 각도를 aligned_wrist로 저장
리프트/하강: aligned_wrist ±0.10~0.15 rad에서만 순응
범위 초과: 경계 안으로 복원 토크 적용
```

### 작업

- 첫 시험은 접촉하지 않는 pre-squeeze에서 멈춰 양팔 자세와 손목 중립을 확인한다.
- 15cm 박스 중심 `[0.45, 0, 1.225]`와 양쪽 표면을 기준으로 접근 목표를 생성한다.
- 시작 관절 명령에서 pre-squeeze 명령까지 5차 보간해 목표 불연속을 없앤다.
- 실제 wrist angle을 매 tick 무제한 목표로 복사하지 않는다.
- soft limit 목표와 직접 복원 토크를 함께 사용한다.
- 접촉력이 양쪽 모두 일정 값 이상일 때만 정렬 판정을 시작한다.
- F/T torque가 일정 시간 낮으면 정렬 완료로 판정한다.

### 통과 조건

- [ ] wrist가 `±0.35 rad` 정상 범위를 크게 벗어나지 않음
- [ ] wrist가 URDF limit `±0.9 rad`에 닿지 않음
- [ ] 양쪽 패드가 물체 측면과 접촉
- [ ] 손목 정렬 후 각도가 안정적으로 유지됨

### Pre-squeeze 결과 기록

```text
Date: 2026-07-19
Box: center [0.45, 0, 1.225], size 0.15m
Targets: Left [0.45, +0.095, 1.225], Right [0.45, -0.095, 1.225]
Clearance: 각 박스 표면에서 20mm, 접촉/힘 제어 비활성
Architecture: 팔당 shoulder/elbow 4DoF로 위치 3DoF, 남는 1DoF는 null-space 팔꿈치 자세
Wrist policy: yaw/pitch는 IK에서 0rad 중립 고정, squeeze 시 제한 순응용으로 예약
Trajectory: 실제 현재 관절각에서 25s 5차 보간, gravity PD target으로 100Hz 전송
Numerical grip error: Left 0.051mm, Right 0.048mm
Actual grip pose after settling: Left [0.450, 0.095, 1.225], Right [0.451, -0.095, 1.226]
Actual height difference: about 1mm
Measured peak velocity: 0.194rad/s
Settled peak velocity: below 7e-6rad/s
Result: 손목 자유도를 소비하지 않고 양손 pre-squeeze 위치에 안정적으로 정지
```

---

## Mission 6 — 힘 및 손목 진단 토픽

### 목표

rqt에서 파지 상태를 즉시 판단할 수 있게 한다.

### 추가 토픽

```text
/dual_arm/grasp_force_left
/dual_arm/grasp_force_right
/dual_arm/grasp_force_target
/dual_arm/wrist_yaw_left
/dual_arm/wrist_yaw_right
/dual_arm/wrist_yaw_target_left
/dual_arm/wrist_yaw_target_right
/dual_arm/admittance_offset_left
/dual_arm/admittance_offset_right
```

### 힘 계산

양손 중심을 잇는 공통 squeeze axis에 world-frame F/T force를 투영한다.

### 통과 조건

- [ ] 모든 진단 토픽이 연속 발행됨
- [ ] rqt_plot에 좌우 힘과 목표 힘 표시
- [ ] rqt_plot에 좌우 wrist 실제/목표 각도 표시
- [ ] 카메라와 힘 그래프를 동시에 관찰 가능

---

## Mission 7 — 제자리 파지 게이트

### 목표

양손 접촉이 안정되기 전에는 리프트가 시작되지 않게 한다.

### 초기 기준값

```text
목표 압착력: 좌우 각각 10 N
파지 인정: 좌우 각각 7.5 N 이상
유지 시간: 0.1~0.2 s
정렬 시작 접촉력: 좌우 각각 2 N 이상
```

### 통과 조건

- [ ] 한쪽만 접촉하면 상승하지 않음
- [ ] 양쪽 힘이 기준을 유지하면 파지 완료
- [ ] 파지 완료 시 wrist 정렬각 저장
- [ ] 힘 손실 시 현재 높이에서 상승 정지

---

## Mission 8 — 수직 LIFT와 HOLD

### 목표

물체의 x/y를 유지하면서 z 방향으로만 들어 올린다.

### 작업

- 기존 수평 transport segment를 사용하지 않는다.
- 초기 상승 높이는 `0.05~0.08 m`로 제한한다.
- 양손 target의 x/y는 파지 완료 위치로 고정한다.
- z만 부드러운 trajectory로 증가시킨다.
- 최고점에서 1~2초 유지한다.

### 통과 조건

- [ ] 물체가 받침대에서 분리됨
- [ ] 상승 중 x/y 이동이 허용 오차 내
- [ ] 물체 roll/pitch/yaw가 과도하게 변하지 않음
- [ ] HOLD 동안 높이와 양손 힘 유지
- [ ] waist/head가 움직이지 않음

---

## Mission 9 — LOWER와 RELEASE

### 목표

물체를 원래 위치로 내려놓고 안전하게 놓는다.

### 작업

- lift trajectory를 역방향으로 실행한다.
- 원래 검출 높이까지 수직 하강한다.
- 하강 완료 전까지 압착력을 유지한다.
- 목표 힘을 약 1초 동안 10 N에서 0 N으로 감소시킨다.
- 양손을 바깥쪽으로 후퇴시킨다.
- wrist를 0 rad로 복귀시킨다.

### 통과 조건

- [ ] 물체가 원래 받침대에 놓임
- [ ] release 전에 물체가 낙하하지 않음
- [ ] release 후 물체 pose가 안정됨
- [ ] 양팔과 wrist가 안전 자세로 복귀

---

## Mission 10 — 반복 시험 및 정리

### 목표

한 번의 성공이 아니라 반복 가능한 box lifting을 완성한다.

### 작업

- 최소 5회 연속 시험한다.
- 각 시험의 검출 pose, 최대 높이, 최소 파지력, 최종 pose를 기록한다.
- launch 하나로 Gazebo와 ArUco detector가 함께 시작되게 한다.
- rqt 설정 또는 진단 실행 방법을 README에 기록한다.
- 임시 진단 코드와 사용하지 않는 place/transport 로직을 정리한다.

### 최종 통과 조건

- [ ] 5회 중 5회 파지 성공
- [ ] 5회 중 5회 목표 높이 도달
- [ ] 5회 중 5회 원래 위치에 내려놓기 성공
- [ ] wrist limit 접촉 0회
- [ ] NaN 및 controller failure 0회
- [ ] 실행 및 검증 절차 문서화 완료

---

## 작업 로그

각 미션 완료 후 아래 형식으로 누적한다.

```text
## YYYY-MM-DD — Mission N

- Commit:
- 변경 파일:
- 빌드 결과:
- Gazebo 결과:
- 측정값:
- 남은 문제:
- 다음 미션 시작 조건:
```

## 2026-07-17 — Mission 0

- Commit: Mission 0 documentation commit (this commit)
- 변경 파일: `BOX_LIFTING_MISSIONS.md`
- 빌드 결과: Release build 성공
- Gazebo 결과: 11DoF 안정, 실제 물체가 목표 근처로 이동 후 정지
- 측정값: 시작 `[0.4500,-0.1500,1.2000]`, 최종 `[0.4404,0.1548,1.2000]`
- 남은 문제: ArUco 중심 오차, 약 4.42 deg 물체 회전, TrajectoryDone 미발행
- 다음 미션 시작 조건: Mission 0 결과 사용자 확인 후 Mission 1 wrist URDF 이식 시작
