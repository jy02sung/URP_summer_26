# 임피던스 기반 실물리 파지 (Kinematic Attach 제거)

- **작성일**: 2026-07-08
- **관련 파일**: `src/main.cpp`, `src/dual_arm_function.h`, `models/aruco_box_26/model.sdf`, `worlds/rrbot.world`
- **배경**: 기존에는 `aruco_box_26`이 `static=true`라서 물리적으로 손에 붙지 않았고, `main.cpp`가 파지~내려놓기 구간 동안 `/gazebo/set_model_state`로 박스 pose를 손 위치에 강제로 갱신하는 "kinematic attach"로 대신했다. 목표는 이 텔레포트 우회책을 없애고, 이미 구현돼 있던 임피던스 제어(가상 스프링-댐퍼 + F/T 센서)가 실제 마찰/접촉만으로 물체를 붙잡아 옮기도록 바꾸는 것.

## 1. Kinematic attach 제거 → 실제 스퀴즈 파지

- `models/aruco_box_26/model.sdf`: `<static>true</static>` → `<static>false</static>` (진짜 동역학 물체로 전환)
- `main.cpp`: `attach_start_row`/`attach_end_row`, `/gazebo/set_model_state` 서비스콜, 관련 include/변수 전부 제거
- 기존에 있던 `PHASE_GRASP_TO_PLACE` 구간의 임피던스 루프(가상 질량-댐핑-강성 `Md/Bd/Kd_imp`, F/T 센서 `F_ext`)는 그대로 유지 — 이게 이제 물체를 붙잡는 유일한 메커니즘이다.
- 원리: 양팔이 `grasp_offset=4.5cm`로 박스 표면(half-width 5cm) 안쪽 5mm를 파고드는 목표 위치를 계속 추종하려 하고, 실제 접촉이 이를 막으면서 생기는 위치오차 `e`에 `Kd_imp*e`가 곧 스퀴즈력이 되어 마찰(mu=1.2)로 물체를 붙잡는다.

## 2. 받침 문제 (박스가 동역학 물체가 되며 발생)

테이블은 이전에 팔 충돌 문제로 이미 제거된 상태였고, 그 부작용(받침 없음)을 kinematic attach로 우회하고 있었다. `static=false`로 바꾸면 파지 전/후에 물체가 자유낙하하므로:

- `worlds/rrbot.world`에 `pick_pedestal`(12x12x10cm, 상판 z=1.15) 추가 — pick 지점(고정 좌표) 바로 아래에서만 파지 전까지 물체를 떠받친다. 예전 테이블처럼 팔 이동 경로 전체를 가로지르지 않아 충돌 재발 우려가 적다.
- place 지점(`transport_pt`)은 실행마다 임의 좌표라 고정 받침대를 두지 않았다 — 내려놓은 후 손을 벌리면 그 자리에 받쳐줄 게 없어 바닥(ground_plane)까지 낙하한다. 특정 위치로 고정해서 쓰게 되면 동일한 방식으로 `place_pedestal`을 추가하면 된다. **(TODO)**

## 3. Pick_pedestal과의 충돌 회피 (standoff 웨이포인트)

받침대를 추가하고 나니, 시작 자세에서 파지점(`objL/R`)으로 곧장 3D 직선 이동하면 z가 받침대 상판(1.15)보다 낮은 구간에서 x,y가 이미 받침대 영역에 들어가 팔이 모서리에 부딪히는 문제가 발생했다. 세 가지 대안(① 오프셋 웨이포인트, ② 장애물 회피 경로계획, ③ 접촉 감지 후 재탐색) 중 기존 아키텍처(웨이포인트별 IK + 직선 세그먼트, 충돌 검사 인프라 없음)에 가장 적합한 ①을 채택:

- `standoffL/R = objL/R + (0, ±0.15, 0)` — 파지 높이(`obj.z`, 받침대 상판보다 5cm 위)는 유지한 채 y만 받침대 반폭(0.06)보다 충분히 크게(0.15) 벌린 지점
- APPROACH를 2단계로 분리: `start_L/R → standoffL/R` (대기 지점 이동) → `standoffL/R → objL/R` (파지 높이를 유지한 채 y로만 직선 접근 — 항상 받침대보다 높은 곳에서만 움직이므로 부딪힐 수 없음)

## 4. 파지 후 수직 리프트

파지 직후 파지점→이송목표로 곧장 대각선 이동하면 바닥/받침대 근처를 스치듯 지나갈 수 있어, 스퀴즈를 유지한 채(`PHASE_GRASP_TO_PLACE`) 먼저 수직으로 들어올린 뒤 이송하도록 분리:

- `liftL/R = objL/R + (0, 0, LIFT_HEIGHT)`, `LIFT_HEIGHT = 0.10m`
- `objL/R → liftL/R` (수직 상승) → `liftL/R → transportL/R` (이송목표로 이동)

## 5. 최종 파이프라인 (5세그먼트, 전부 `buildGraspPipelineFromDetection` 내 생성)

| # | 구간 | TaskPhase | 비고 |
|---|------|-----------|------|
| 1 | `start_L/R → standoffL/R` | PHASE_APPROACH | 받침대 바깥 대기 지점으로 이동 |
| 1b | `standoffL/R → objL/R` | PHASE_APPROACH | 파지 높이 유지한 채 y축 직선 접근 |
| 2 | `objL/R → liftL/R` | PHASE_GRASP_TO_PLACE | 스퀴즈 유지한 채 수직 리프트, 임피던스 ON |
| 2b | `liftL/R → transportL/R` | PHASE_GRASP_TO_PLACE | 들어올린 높이에서 이송목표로 이동 |
| 3 | `transportL/R → start_L/R` | PHASE_RETURN | 내려놓고 원위치 복귀, 임피던스 OFF |

`pos_segs`/`seg_phase`는 원래 가변 개수로 짜여 있어 세그먼트 추가로 다른 곳이 깨지지 않는다.

## 6. 미해결/추후 확인 필요

- **컨택트 물리 검증**: 스퀴즈력만으로 버티는지는 Gazebo 컨택트 솔버 특성(강성/댐핑)에 좌우돼 코드 분석만으로는 확답 못함. 대략 계산상 스퀴즈력(~1.5N/arm) 대비 필요 마찰력(~0.82N/side)은 여유가 있으나, 미끄러지면 `Kd_imp_left/right`(현재 300 N/m, `dual_arm_function.h:131,135`)를 올리거나 `grasp_offset`을 더 조이는 방향으로 튜닝 필요.
- **LIFT_HEIGHT(0.10m)**: 실측 후 여유 부족하면 조정.
- **place_pedestal**: place 지점이 특정 좌표로 고정되면 추가 필요 (현재는 미지원, 낙하함).

---

# Gazebo 실행 방법 (간단 요약)

## 사전 준비
```bash
cd ~/friend_ws
catkin_make
source devel/setup.bash
```

## 1) Gazebo + 로봇 스폰 + 제어 노드 실행
```bash
roslaunch dual_arm gazebo.launch
```
- `worlds/rrbot.world`를 로드 (ground_plane, aruco_box_26, aruco_marker_27, pick_pedestal 포함)
- `dual_arm.xacro`를 스폰하고 effort controller들을 기동
- `dual_arm_main`(`src/main.cpp`) 노드를 같이 실행 — 관절 상태 구독, RNEA 토크 계산, 궤적 재생을 담당하는 메인 제어 루프(1000Hz)

## 2) ArUco 인식 노드 실행 (별도 터미널)
```bash
roslaunch dual_arm aruco_detection.launch
```
- `aruco_ros/single`이 `Head_camera` 이미지를 구독해 marker id=26 pose를 `/aruco_ros/pose`로 발행
- `dual_arm_main`이 이걸 구독해 world 좌표로 변환(TF) 후 파지 파이프라인 생성에 사용

## 3) 명령 입력 노드 실행 (별도 터미널)
```bash
rosrun dual_arm dual_arm_command
```
대화형 CLI로 `/dual_arm/DualArmCmd_sim`을 발행한다. 최상위 메뉴에서 `vision`을 선택하면:
1. Head가 자동으로 스캔 자세(yaw=0, pitch=+30°)로 이동해 마커를 찾는다 (연속 3프레임 검출로 확정)
2. `0`을 입력해 이송 목표 위치 `[x y z]`(world frame, meter)를 입력
3. 위 5세그먼트 파이프라인이 자동 생성되어 재생됨 (접근→파지→리프트→이송→복귀)

`modeling`/`simulation` 메뉴는 vision pick과 무관한 저수준 테스트용 모드(관절각 직접 입력, EE 좌표 직선/역기구학 테스트)다.

## 4) (선택) RViz
```bash
roslaunch dual_arm rviz.launch
```

## 참고
- 로봇/제어 관련 기본 실행법은 `README.md`에도 정리돼 있음 (의존 패키지 설치 등)
- 전체 기능 현황/설계 배경은 `PRD.md` 참고
