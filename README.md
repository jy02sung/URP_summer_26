# Dual Arm Humanoid Robot — Vision-based Pick & Place

듀얼암 휴머노이드 로봇의 비전 기반 물체 파지 및 이송 시스템.
ArUco 마커 인식 → 양팔 동시 파지 → 임피던스 제어로 실제 스퀴즈 파지를 유지하며 이송까지의 전체 파이프라인 구현.

**GitHub**: https://github.com/jy02sung/URP_summer_26/tree/jmp
**워크스페이스**: `~/friend_ws`
**Author**: Seungjun Lee (sjun0107@g.skku.edu)

- ✅ Head 11DoF 추가 (기존 9DoF + Head_yaw/Head_pitch)
- ✅ 카메라 마운트 (RGB-D)
- ✅ ArUco 인식 → 3D 좌표
- ✅ TF 변환 (카메라 → world)
- ✅ 양팔 동시 파지 궤적
- ✅ 팔별 독립 Tf 계산 (한쪽 팔 변위가 작을 때 다른 팔에 끌려 늦게 출발하는 문제 해결)
- ✅ 임피던스 제어로 실물리 스퀴즈 파지 (파지~내려놓기, kinematic attach 아님)
- ✅ TaskPhase 자동 전환
- ✅ standoff/lift 웨이포인트로 받침대 충돌 회피

---

## 시스템 구성

```
Head 카메라 (RGB-D)
       ↓
ArUco 마커 인식 (id=26)
       ↓
3D 좌표 계산 (TF 변환: 카메라 → world)
       ↓
Cartesian IK (DLS) → 양팔 동시 직선 궤적 생성 (standoff → 파지 → 리프트 → 이송)
       ↓
임피던스 제어(가상 스프링-댐퍼 + F/T 센서)로 스퀴즈 파지 유지 → 이송 → 내려놓기 → 복귀
```

---

## 환경

- Ubuntu 20.04 + ROS Noetic
- Gazebo 11
- Pinocchio (`robotpkg-py38-pinocchio`)
- Eigen 3.4.0
- aruco_ros

### Dependencies 설치

```bash
sudo apt update
sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers \
    ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install robotpkg-py38-pinocchio
```

---

## 실행 방법

### 사전 준비

```bash
cd ~/friend_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 터미널 1 — ROS 마스터

```bash
roscore
```

### 터미널 2 — Gazebo + 로봇 스폰 + 제어 노드

```bash
roslaunch dual_arm gazebo.launch
```

- `worlds/rrbot.world` 로드 — `ground_plane`, `aruco_box_26`(동역학 물체), `pick_pedestal`(pick 지점 전용 소형 받침대), `aruco_marker_27`(이송 목표 시각 랜드마크) 포함
- `dual_arm.xacro` 스폰 + effort controller 기동
- `dual_arm_main` 노드 실행 (1000Hz 제어 루프)

### 터미널 3 — ArUco 인식 노드

```bash
roslaunch dual_arm aruco_detection.launch
```

- `aruco_ros/single`이 Head 카메라 이미지 구독
- marker id=26 pose를 `/aruco_ros/pose`로 발행

### 터미널 4 — 명령 입력

```bash
rosrun dual_arm dual_arm_command
```

---

## Vision Pick & Place 실행

```
vision       ← vision 모드 선택
0            ← 이송 목표 좌표 입력 선택
0.45 0.4 1.20  ← 이송 목표 [x y z] (world frame, meter)
```

Head가 자동으로 스캔 자세(yaw=0, pitch=+0.5236rad)로 이동해 마커를 찾고, 연속 3프레임 일치로 검출을 확정한 뒤 아래 파이프라인이 자동 실행된다.

---

## 파지 파이프라인 (5세그먼트)

| # | 구간 | TaskPhase | 내용 |
|---|------|-----------|------|
| 1 | 현재위치 → 대기지점(standoff) | APPROACH | 파지 높이를 유지한 채 받침대 바깥쪽(y로 0.15m)으로 이동 |
| 1b | 대기지점 → 파지위치 | APPROACH | 파지 높이(obj.z)를 유지한 채 y축 직선 접근 — 받침대보다 높은 위치에서만 움직여 충돌 없음 |
| 2 | 파지위치 → 리프트 | GRASP_TO_PLACE | 파지 높이에서 수직으로 0.10m 상승(`LIFT_HEIGHT`, 절대 높이 아닌 상대값), 임피던스 ON |
| 2b | 리프트 → 이송목표 | GRASP_TO_PLACE | 리프트 높이에서 이송목표로 직선 이동 (목표 z가 다르면 대각선 하강 포함) |
| 3 | 이송목표 → 원위치 | RETURN | 내려놓기 후 복귀, 임피던스 OFF |

물체는 양팔이 `grasp_offset=0.040m`로 박스 표면(half-width 0.05m) 안쪽 10mm를 스퀴즈한 채로 세그먼트 2~2b 내내 임피던스 제어를 유지하는 것만으로 붙잡힌다 — kinematic attach(강제 pose 갱신)는 쓰지 않는다.

---

## 핵심 메커니즘

### 1. 11DoF 구성 (Head 2DoF 추가)

```
기존 9DoF: Waist(1) + 왼팔(4) + 오른팔(4)
변경 11DoF: Waist(1) + Head_yaw(1) + Head_pitch(1) + 왼팔(4) + 오른팔(4)
```

- Head_joint (고정) → Head_yaw_joint + Head_pitch_joint (revolute)
- Head에 RGB-D 카메라 마운트
- Pinocchio 알파벳순 정렬로 관절 인덱스 재매핑

### 2. Head 자동 스캔

```
vision 실행 → Head pitch=+0.5236rad(30°) 고정 이동
→ 0.5s 정지 대기 후 ArUco 마커 인식 확인 (연속 3프레임, 1cm 이내 일치 시 확정, 최대 4s 타임아웃)
→ 파지 파이프라인 자동 시작
```

- 격자 스캔은 구현돼 있지 않다 — 고정된 한 방향(yaw=0, pitch=+30°)만 보고 검출을 시도한다.
- pitch +(양수)가 아래(물체 쪽) 방향 (직관과 반대, 실측으로 확인된 값).

### 3. Cartesian IK (DLS)

손끝 목표 위치(x, y, z) → 관절 각도 자동 계산 (양팔 6D 스택 태스크 동시 해석)

```
오차 e = [목표L - 현재L ; 목표R - 현재R]  (6x1)
DLS 역행렬: Jᵀ (J·Jᵀ + λ²I)⁻¹,  λ=0.1
관절각 업데이트: q += step × Jᵀ(JJᵀ+λ²I)⁻¹ e,  step=1.0, tol=1e-4m, maxIter=300
```

- 감쇠항 λ² 덕분에 특이점 근방에서도 안정적으로 수렴.

### 4. Cartesian 직선 궤적

관절 공간 보간 대신 손끝이 3D 직선으로 이동 (5차 시간 스케일링, `v_des=0.1m/s` 기준 소요시간 산정).

- 파지 접근/이송 각 세그먼트를 개별 직선으로 잇는 방식이라, 장애물(받침대)을 피해야 하는 구간은 waypoint를 추가해 여러 개의 직선으로 나눈다 (위 5세그먼트 표 참고). 궤적 자체에 충돌 회피 로직은 없다 — 순전히 웨이포인트 배치로 회피한다.
- standoff/lift 높이는 모두 파지 높이(`obj.z`) 기준 **상대값**이다 (`STANDOFF_Y=0.15m`, `LIFT_HEIGHT=0.10m`). 고정된 절대 안전고도(예: z=1.50) 방식은 쓰지 않는다.

### 5. 계산토크(Computed Torque) 제어

```cpp
// PDController()는 위치/속도 오차로부터 "보정 가속도"를 만든다 (토크가 아님)
qddot_cmd = 궤적 feedforward 가속도 + PD_acc
          [+ 임피던스 보정 가속도 (GRASP_TO_PLACE 구간만)]

target_torque = pinocchio::rnea(q, qdot, qddot_cmd)   // 관성 + 코리올리 + 중력 전부 포함
```

- `computeGeneralizedGravity()`로 별도 중력토크를 계산해두긴 하지만 실제로 더해지진 않는다 — 최종 토크는 `rnea()` 한 번으로 관성/코리올리/중력을 통째로 반영한 결과를 그대로 쓴다 (PD+중력보상을 따로 더하는 방식이 아니다).

### 6. 임피던스 제어 (실물리 스퀴즈 파지)

파지~이송 구간(`PHASE_GRASP_TO_PLACE`)에서 활성화, F/T 센서 기반 가상 스프링-댐퍼-질량:

```
Md·e_ddot + Bd·e_dot + Kd_imp·e = F_ext     (e = x_actual - x_desired, world frame 위치 3축)

F_ext: F/T 센서 (/dual_arm/left_ft_sensor, /dual_arm/right_ft_sensor), 10Hz 로우패스 필터 통과 후 사용
       (원시값은 접촉 순간 노이즈가 커서 필터 없이 쓰면 그대로 토크에 증폭되어 접촉이 떨림)
e_ddot → DampedPinv(J, λ=0.05)로 관절가속도 변환 → qddot_cmd에 additive
```

- 양팔이 `grasp_offset` 만큼 물체 표면 안쪽을 파고드는 목표 위치를 계속 추종하려 하고, 실제 접촉이 막아서 생기는 위치오차(e)에 `Kd_imp`가 곱해진 만큼이 곧 스퀴즈력이 되어 마찰(mu=1.2)로 물체를 붙잡는다. **박스 pose를 강제로 갱신하는 kinematic attach는 더 이상 쓰지 않는다.**

### 7. TaskPhase 자동 전환

```
PHASE_APPROACH (0)       → 접근 구간, 임피던스 OFF
PHASE_GRASP_TO_PLACE (1) → 파지~리프트~이송, 임피던스 ON
PHASE_RETURN (2)         → 복귀, 임피던스 OFF
PHASE_SCAN (3)           → Head 스캔 구간 (수동 오버라이드 대상 아님)
```

### 8. 받침대 (pick_pedestal)

`aruco_box_26`은 동역학(dynamic) 물체라 받침이 없으면 자유낙하한다. pick 지점(고정 좌표) 바로 아래에만 물체 발판 크기의 작은 정적 받침대(`pick_pedestal`, 12x12x10cm, 상판 z=1.15)를 둬서 파지 전까지만 지지한다 — 예전에 팔 충돌 문제로 제거했던 전체 테이블과 달리 팔 이동 경로 전체를 가로지르지 않는다. place 지점은 실행마다 임의 좌표라 별도 받침대가 없다 — 내려놓은 뒤 받쳐줄 게 없으면 바닥까지 낙하한다 (place 지점을 고정해서 쓰게 되면 동일한 방식으로 추가 필요).

---

## 패키지 구조

```
Dual_Arm_ROS_Simulation/
├── src/
│   ├── main.cpp              # 메인 제어 루프 (1000Hz)
│   ├── dual_arm_command.cpp  # 명령 입력 CLI
│   ├── dual_arm_function.cpp # 궤적/제어 함수
│   └── dual_arm_function.h   # 파라미터, 게인, enum
├── urdf/
│   ├── dual_arm.urdf         # 11DoF URDF (Head 2DoF 포함)
│   └── dual_arm.xacro
├── launch/
│   ├── gazebo.launch
│   └── aruco_detection.launch
├── config/
│   ├── dual_arm_effortcontrol.yaml
│   └── dual_arm_positioncontrol.yaml
├── worlds/
│   └── rrbot.world           # ground_plane, aruco_box_26(동역학), pick_pedestal, aruco_marker_27 포함 (테이블 없음)
└── models/
    ├── aruco_box_26/         # 파지 대상 박스 (id=26, 동역학, mu=1.2)
    └── aruco_marker_27/      # 이송 목표 마커 (id=27, 시각 랜드마크)
```

---

## 게인 설정

```cpp
// dual_arm_function.h
Kp = { 1000, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 }
Kd = { 50,   30,  30,  1,   1,   1,   1,   1,   1,   1,   1   }
// 순서: Waist, Head_yaw, Head_pitch, L팔4개, R팔4개

// 임피던스 파라미터 (좌/우 동일, 위치 3축 공통)
Md = 2.0kg, Bd = 65.0 N·s/m, Kd_imp = 500.0 N/m
IMPEDANCE_DLS_LAMBDA = 0.05   // 임피던스 가속도 -> 관절가속도 변환용 댐핑 계수
```

---

## 기타

- 자세한 설계 배경/기능 현황: `PRD.md`
- kinematic attach 제거 및 받침대/standoff/lift 도입 과정 상세: `IMPEDANCE_GRASP.md`
- Gazebo 종료: `killall -9 gzclient gzserver roslaunch rosmaster`
