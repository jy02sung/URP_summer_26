✅ Head 11DoF 추가
✅ 카메라 마운트 (RGB-D)
✅ ArUco 인식 → 3D 좌표
✅ TF 변환 (카메라 → world)
✅ 양팔 동시 파지 궤적
✅ Tf 독립 계산 (딜레이 해결)
✅ 임피던스 제어 (파지~내려놓기)
✅ TaskPhase 자동 전환
✅ 이송 오차 감소 확인
# Dual Arm Humanoid Robot — Vision-based Pick & Place

듀얼암 휴머노이드 로봇의 비전 기반 물체 파지 및 이송 시스템.  
ArUco 마커 인식 → 양팔 동시 파지 → 임피던스 제어 이송까지 전체 파이프라인 구현.

**GitHub**: https://github.com/jy02sung/URP_summer_26/tree/jmp  
**워크스페이스**: `~/friend_ws`

---

## 시스템 구성

```
Head 카메라 (RGB-D)
       ↓
ArUco 마커 인식 (id=26)
       ↓
3D 좌표 계산 (TF 변환: 카메라 → world)
       ↓
Cartesian IK (DLS) → 양팔 동시 궤적 생성
       ↓
임피던스 제어로 파지 → 이송 → 내려놓기 → 복귀
```

---

## 환경

- Ubuntu 20.04 + ROS Noetic
- Gazebo 11
- Pinocchio (`robotpkg-py38-pinocchio`)
- Eigen 3.4.0
- aruco_ros

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

- `worlds/rrbot.world` 로드 (aruco_box_26, aruco_marker_27 포함)
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

---

## 파지 파이프라인 (5세그먼트)

| # | 구간 | TaskPhase | 내용 |
|---|------|-----------|------|
| 1 | 현재위치 → 대기지점 | APPROACH | 물체 옆 안전 지점으로 이동 |
| 1b | 대기지점 → 파지위치 | APPROACH | y축 직선 접근 |
| 2 | 파지위치 → 리프트 | GRASP_TO_PLACE | 수직 상승, 임피던스 ON |
| 2b | 리프트 → 이송목표 | GRASP_TO_PLACE | 안전높이(z=1.50) 유지 수평 이동 |
| 3 | 이송목표 → 원위치 | RETURN | 내려놓기 후 복귀, 임피던스 OFF |

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
vision 실행 → Head pitch=+0.52 고정 이동
→ ArUco 마커 인식 대기 (연속 3프레임 일치 시 확정)
→ 파지 파이프라인 자동 시작
```

- `SCAN_GRID_ENABLED = false`: 격자 스캔 비활성, 고정 방향으로 빠른 인식
- pitch +(양수)가 아래 방향 (직관과 반대)

### 3. Cartesian IK (DLS)

손끝 목표 위치(x, y, z) → 관절 각도 자동 계산

```
오차 = 목표위치 - 현재위치
DLS 역행렬: (J·Jᵀ + λ²I)⁻¹·Jᵀ
관절각 업데이트: q += step × DLS_inv × 오차
```

- 특이점에서도 안정적 (감쇠항 λ² 적용)

### 4. Cartesian 직선 궤적

관절 공간 보간 대신 손끝이 직선으로 이동:

```
수평 이동: z=1.50 고정, xy만 변화
수직 이동: z축만 변화
→ 테이블 충돌 방지 (테이블 최고 높이 1.15보다 35cm 위)
```

### 5. PD + 중력보상 제어

```
토크 = Kp × (목표각도 - 현재각도)
      + Kd × (목표속도 - 현재속도)
      + 중력토크 (Pinocchio computeGeneralizedGravity)
```

### 6. 임피던스 제어

파지~이송 구간(GRASP_TO_PLACE)에서 활성화:

```
Md·x_ddot + Bd·x_dot + Kd·(x - x_d) = F_ext

F_ext: F/T 센서 (/dual_arm/left_ft_sensor, /dual_arm/right_ft_sensor)
→ 기존 PD+RNEA 토크에 additive하게 반영
```

### 7. TaskPhase 자동 전환

```
PHASE_APPROACH (0)     → 접근 구간, 임피던스 OFF
PHASE_GRASP_TO_PLACE (1) → 파지~이송, 임피던스 ON
PHASE_RETURN (2)       → 복귀, 임피던스 OFF
PHASE_SCAN (3)         → Head 스캔 구간
```

### 8. Kinematic Attach

파지 구간에서 박스가 팔을 따라 이동:

```
양팔 EE 중점 → /gazebo/set_model_state로 박스 위치 갱신 (50Hz)
```

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
│   └── rrbot.world           # 테이블, ArUco 박스 포함
└── models/
    ├── aruco_box_26/         # 파지 대상 박스 (id=26)
    └── aruco_marker_27/      # 이송 목표 마커 (id=27)
```

---

## 게인 설정

```cpp
// dual_arm_function.h
Kp = { 1000, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 }
Kd = { 50,   30,  30,  1,   1,   1,   1,   1,   1,   1,   1   }
// 순서: Waist, Head_yaw, Head_pitch, L팔4개, R팔4개

// 임피던스 파라미터
Md = 2.0, Bd = 50.0, Kd_imp = 300.0
```

---


- Gazebo 종료: `killall -9 gzclient gzserver roslaunch rosmaster`