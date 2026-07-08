# PRD: Dual-Arm Vision-Guided Pick & Place

- **작성일**: 2026-07-08
- **워크스페이스**: `~/friend_ws/src/Dual_Arm_ROS_Simulation`
- **GitHub**: https://github.com/jy02sung/URP_summer_26/tree/jmp (branch: `jmp`)
- **환경**: Ubuntu 20.04 + ROS Noetic, Gazebo, Pinocchio(RNEA/FK/Jacobian), aruco_ros

## 1. 목표

사용자가 물체 좌표/특성(무게, 크기)과 목표 위치를 입력하면, 양팔 로봇이 아래 과정을 완전 자동으로 수행한다.

1. Head를 자동 스캔해 카메라로 물체를 탐색
2. ArUco 마커로 물체의 3D 좌표를 자동 계산
3. 양팔이 동시에 접근해 물체를 파지
4. 임피던스 제어로 물체 특성(무게·크기)에 따라 파지력을 자동 조절
5. 목표 위치로 이송
6. 내려놓고 초기 자세로 복귀

현재는 이 파이프라인 중 **"물체 좌표 입력 → 자동 실행"** 구간(3~5, 부분적으로 2)까지 구현되어 있고, **입력 자동화(1, 사용자 파라미터화)와 인식 안정성**이 남은 핵심 과제다.

## 2. 시스템 개요

- **DoF 구성 (11DoF, `main.cpp:38`)**: `0:Waist, 1:Head_yaw, 2:Head_pitch, 3~6:L arm(shoulder_pitch/roll/yaw, elbow), 7~10:R arm(동일)`
- **제어 모드**: Effort control 기반 RNEA 토크 제어 (`ARMCTRLMODE=EFFORT`, `main.cpp:33`). Position control 백엔드도 병존.
- **명령 인터페이스**: `dual_arm_command` 노드(CLI 대화형) → `/dual_arm/DualArmCmd_sim` (`std_msgs/Float32MultiArray`) 발행 → `main.cpp`가 구독해 `command_mode`별로 분기
  - mode 0: modeling(관절각 직접 입력)
  - mode 1: joint sim (IK 1회 + 관절공간 5차 궤적)
  - mode 2: cartesian sim (직교 직선 경로, 매 스텝 IK)
  - mode 3: **vision pick** (ArUco 인식 → world 변환 → 접근/파지/이송/복귀 자동 궤적)
- **IK**: DLS(Damped Least Squares), 좌우 EE 6D(3+3) 스택 태스크 동시 해석, λ=0.1, tol=1e-4m, maxIter=300 (`dual_arm_function.cpp:188`)
- **궤적**: Quintic polynomial, 팔별 독립 Tf(도달시간=변위/0.5rad/s) 계산 (`dual_arm_function.cpp:74`)

---

## 3. Feature 1 — Head 스캔 & 카메라 인식

### 구현 상태: 🟡 부분 구현 (하드웨어/센서만 완료, 자동 스캔 로직 없음)

| 항목 | 상태 |
|---|---|
| Head 2DoF (yaw/pitch) 관절 및 액추에이터 | ✅ 완료 |
| Head 마운트 RGB-D 카메라 링크/조인트 | ✅ 완료 |
| Head 관절 상태 피드백(`/dual_arm/joint_states`) | ✅ 완료 |
| **자동 스캔(탐색 패턴) 로직** | ❌ 미구현 |
| **"물체가 시야에 없을 때" 판단/재탐색** | ❌ 미구현 |

### 기술 스펙
- 관절: `Head_yaw_joint` (`urdf/dual_arm.xacro:740`) — limit ±1.2217 rad (**±70°**), effort 10, velocity 8.5
- 관절: `Head_pitch_joint` (`urdf/dual_arm.xacro:759`) — limit ±0.5236 rad (**±30°**), effort 10, velocity 8.5
- 카메라: `Head_camera_link` (fixed joint, `Head_camera_link_optical` 프레임 존재) — RGB(`/dual_arm/head_camera/rgb/image_raw`, `.../camera_info`) 스트림을 aruco_ros가 구독
- 현재는 Head가 고정 자세(0,0)로 있는 상태에서만 인식이 이뤄짐 — 물체가 카메라 시야각 밖에 있으면 인식 자체가 불가능

### 다음 할 일
1. Head yaw/pitch를 격자(예: yaw -70°~+70°, pitch -30°~+30°)로 스텝 이동시키는 스캔 궤적/상태머신 추가
2. 각 스캔 스텝에서 일정 시간 대기 후 `/aruco_ros/pose` 수신 여부 확인 → 검출되면 스캔 중단하고 다음 단계(3D 좌표 계산)로 전환
3. 스캔 완료까지 물체를 못 찾을 경우의 실패 처리(재시도/에러 보고) 정의
4. `main.cpp`의 `command_mode==3` 진입 전 단계로 `PHASE_SCAN` 같은 새 TaskPhase 추가 검토

---

## 4. Feature 2 — ArUco 마커 기반 3D 좌표 자동 계산

### 구현 상태: 🟡 부분 구현 (물체 마커는 동작, 목표지점 마커는 미연동, 알려진 버그 있음)

| 항목 | 상태 |
|---|---|
| 물체 마커(id=26) Gazebo 모델 + aruco_ros 검출 launch | ✅ 완료 |
| 목표지점 마커(id=27) Gazebo 모델(시각적 오브젝트) | ✅ 존재 (모델만) |
| **목표지점 마커(id=27) 실제 검출/좌표 획득** | ❌ 미구현 (현재는 CLI로 수동 xyz 입력) |
| 카메라 프레임 → world 프레임 TF 변환 | ✅ 완료 |
| **"ArUco 인식 버그"** | ❌ 원인 미특정 — 재현/수정 필요 |

### 기술 스펙
- `aruco_ros/single` 노드 1개만 실행 중 (`launch/aruco_detection.launch`), `markerId=26` 고정, 이미지 토픽을 `/dual_arm/head_camera/rgb/image_raw`로 remap
- 마커 크기 0.05m, `min_marker_size=0.005`(기본 0.02보다 완화하여 원거리 마커 필터링 방지 목적으로 조정된 상태)
- 검출 결과: `/aruco_ros/pose` (`geometry_msgs/PoseStamped`, `Head_camera_link_optical` 기준) → `main.cpp:9` 콜백에서 수신
- world 변환: `tf::TransformListener::transformPose("world", ..., ros::Time(0))` (`main.cpp:369-381`), 실패 시 `command_mode==3` 궤적 생성을 스킵(TF 예외 캐치)
- **목표지점(id=27)은 현재 인식 파이프라인에 전혀 연결되어 있지 않음** — `dual_arm_command.cpp`의 vision 화면(`SCREEN_VISION_TARGET`)에서 사용자가 이송 목표 xyz를 직접 타이핑해서 `/dual_arm/DualArmCmd_sim`의 `data[1..3]`으로 보냄. "목표 위치를 입력"이라는 최종 목표와는 부합하지만, "ArUco로 목표지점 자동 계산"은 아직 없음.

### 다음 할 일
1. ArUco 인식 버그 재현 및 원인 파악(오검출/좌표 튐/검출 실패 빈도 등 증상 구체화 필요 — 로그·rosbag 확보 권장)
2. id=27용 두 번째 `aruco_ros/single` 인스턴스(또는 멀티마커 검출 방식)를 추가해 목표지점 좌표도 자동 획득
3. 물체/목표 마커를 동시에 안정적으로 구분 인식(토픽 네임스페이스 분리, 예: `/aruco_object/pose`, `/aruco_target/pose`)
4. 인식 신뢰도 체크(연속 N프레임 일치 시에만 좌표 확정 등) 추가해 노이즈로 인한 오동작 방지

---

## 5. Feature 3 — 양팔 동시 파지 궤적

### 구현 상태: ✅ 완료 (물체 크기 고정 가정 하에서)

| 항목 | 상태 |
|---|---|
| 좌우 EE 6D 스택 IK (DLS) | ✅ 완료 |
| 접근 → 파지 → 들어올리기 → 이송 → 내려놓기 → 복귀 6세그먼트 궤적 | ✅ 완료 |
| Quintic 궤적 시간 스케일링(팔별 독립 Tf) | ✅ 완료 |
| 팔 간격(straddle/grasp offset) | ⚠️ 하드코딩 |

### 기술 스펙 (`main.cpp:362-471`, `command_mode==3`)
- 물체를 y축 양쪽에서 감싸는 방식으로 파지 (10cm 정육면체, `aruco_box_26` 기준 하드코딩)
  - `straddle_offset = 0.08m` — 접근 시 벌린 간격
  - `grasp_offset = 0.045m` — 파지 시 좁힌 간격 (표면 안쪽 5mm 압착)
  - `lift_offset = 0.10m` — 들어올리는 높이
- 세그먼트별 TaskPhase 태깅: 접근(`PHASE_APPROACH`) → 파지~내려놓기(`PHASE_GRASP_TO_PLACE`, 4개 세그먼트) → 복귀(`PHASE_RETURN`)
- 세그먼트 간 시드(`seed_vec`) 연속성 유지로 IK 해가 급격히 튀지 않도록 함
- 궤적 실행 완료 시 `/dual_arm/TrajectoryDone`(`std_msgs/Bool`) 1회 발행, `/dual_arm/TaskPhase`(`std_msgs/Int32`)는 전환 시점마다 발행(edge-trigger)

### 다음 할 일
1. `straddle_offset`/`grasp_offset`을 물체 크기 입력값에서 동적으로 계산하도록 변경 (현재는 10cm 정육면체 전용 상수)
2. 물체 크기·형상이 바뀔 때 파지 접근 방향(현재 y축 고정)을 일반화할지 검토

---

## 6. Feature 4 — 임피던스 제어 (파지력 자동 조절)

### 구현 상태: 🟡 부분 구현 (제어기 자체는 동작, "물체 특성 기반 자동 조절"은 미구현)

| 항목 | 상태 |
|---|---|
| Cartesian 임피던스 제어기 (Md/Bd/Kd, F/T 센서 피드백) | ✅ 완료 |
| PHASE_GRASP_TO_PLACE 구간에서만 자동 활성화 | ✅ 완료 |
| **물체 무게·크기 입력 → Md/Bd/Kd 자동 매핑** | ❌ 미구현 (현재 상수 고정) |

### 기술 스펙 (`main.cpp:538-593`)
- 모델: `Md*e_ddot + Bd*e_dot + Kd*e = F_ext` (world frame, 위치 3축, 좌우 독립)
- 현재 고정값 (`main.cpp:127-133`):
  - `Md_left/right = [2.0, 2.0, 2.0] kg`
  - `Bd_left/right = [50.0, 50.0, 50.0] N·s/m`
  - `Kd_imp_left/right = [300.0, 300.0, 300.0] N/m`
- F/T 센서: `/dual_arm/left_ft_sensor`, `/dual_arm/right_ft_sensor` (`geometry_msgs/WrenchStamped`) → EE 프레임 회전으로 world frame 변환
- 가상 응답 가속도(`e_ddot`)를 댐핑 의사역행렬(`DampedPinv`, λ=0.05)로 관절가속도化 → RNEA 입력에 가산되어 최종 토크에 반영
- 활성화 조건: `task_phase == PHASE_GRASP_TO_PLACE`일 때만 (접근/복귀 구간은 순수 PD+RNEA)

### 다음 할 일
1. 물체 무게(mass) → 목표 파지력/강성(Kd) 매핑 정책 정의 (예: 무거울수록 Kd↑ Bd↑로 처짐 최소화, 혹은 부서지기 쉬운 물체는 Kd↓)
2. 물체 크기 → 파지 접근/압착 오프셋(Feature 3)과 연동
3. 사용자 입력(무게, 크기, 재질 등) 파라미터를 받아 `Md/Bd/Kd_imp_{left,right}`를 런타임에 계산하는 함수 추가 (현재는 전역 상수로 고정, 재컴파일 없이 조절 불가)
4. 파지력 상한/하한 안전 범위 설정 (과도한 힘으로 인한 물체 파손·미끄러짐 방지)

---

## 7. Feature 5 — TaskPhase 자동 전환 & 전체 파이프라인 자동화

### 구현 상태: 🟡 부분 구현 (수동 트리거 → 세그먼트 자동 재생까지는 완료, 처음부터 끝까지 완전 자동은 아님)

| 항목 | 상태 |
|---|---|
| TaskPhase enum 및 세그먼트별 자동 전환 | ✅ 완료 |
| `/dual_arm/TaskPhase`, `/dual_arm/TrajectoryDone` 발행(관측용) | ✅ 완료 |
| 수동 오버라이드(idle/mode 0·1·2에서 `/dual_arm/TaskPhase` 구독) | ✅ 완료 |
| **Head 스캔 → 인식 → 파지 → 이송 → 복귀 원샷 자동 실행** | ❌ 미구현 (현재는 CLI로 mode=3 진입 + 목표 xyz 수동 입력이 트리거) |

### 기술 스펙
- `enum TaskPhase { PHASE_APPROACH=0, PHASE_GRASP_TO_PLACE=1, PHASE_RETURN=2 }` (`dual_arm_function.h:119`)
- `main.cpp` 재생 루프가 `traj_cnt` 기준으로 `dual_arm_phase_trajectory`를 읽어 현재 phase를 매 스텝 갱신 → 임피던스 on/off의 근거로 사용
- 궤적 종료 시 phase를 `PHASE_APPROACH`로 리셋 + `TrajectoryDone` 1회 발행

### 다음 할 일
1. 현재 "사용자가 CLI에서 vision 모드 선택 + 이송목표 xyz 입력"으로 트리거되는 구조를, "물체 특성+목표좌표 1회 입력 → Head 스캔부터 복귀까지 전자동" 흐름으로 재구성
2. `PHASE_SCAN`(Head 자동 스캔), `PHASE_DETECT`(좌표 계산) 등 앞단 phase를 TaskPhase enum에 추가하고 상태머신으로 연결
3. 실패 처리(인식 실패, IK 미수렴, TF 실패 등) 시 안전한 정지/재시도 정책 수립

---

## 8. 사용자 입력 인터페이스 (현재 vs 목표)

| | 현재 | 목표 |
|---|---|---|
| 물체 좌표 | ArUco(id=26) 자동 인식 (Head 고정 자세 시야 내에 있을 때만) | Head 자동 스캔 + ArUco 자동 인식 |
| 물체 특성(무게/크기) | 미입력, 파지 오프셋·임피던스 파라미터 모두 상수 | 사용자 입력 → 파지 오프셋/임피던스 파라미터 자동 계산 |
| 목표 위치 | CLI(`dual_arm_command`)에서 xyz 수동 입력 | ArUco(id=27) 자동 인식 또는 좌표 직접 입력 |
| 실행 트리거 | CLI에서 vision 모드 선택 | 위 입력 완료 시 자동 실행 |

---

## 9. 우선순위 제안 (다음 스프린트)

1. **ArUco 인식 버그 수정** — 다른 모든 기능의 신뢰성 전제조건
2. **Head 자동 스캔** — "자동 인식" 목표의 핵심 누락 부분
3. **물체 특성 → 임피던스 파라미터 자동 조절** — 현재 상수를 함수화
4. **목표지점 마커(id=27) 자동 인식** 또는 좌표 직접입력 UX 확정
5. **전체 파이프라인 통합 자동화** (위 항목들이 끝난 뒤 상태머신으로 연결)
