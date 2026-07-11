# Vision Pick-Place Workflow

이 문서는 `~/catkin_ws`의 현재 dual-arm Gazebo 데모가 어떻게 작동하는지, 그리고 이번에 안정화된 포인트가 무엇인지 정리한 것이다.

## 대상 파일

- `src/Dual_Arm_ROS_Simulation/src/main.cpp`
- `src/Dual_Arm_ROS_Simulation/src/dual_arm_function.cpp`
- `src/Dual_Arm_ROS_Simulation/urdf/dual_arm.xacro`
- `src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf`
- `src/Dual_Arm_ROS_Simulation/worlds/rrbot.world`
- `src/Dual_Arm_ROS_Simulation/models/aruco_box_26/model.sdf`

## 현재 동작 순서

1. Gazebo가 로봇, 큐브, pedestal, head camera, `aruco_ros`를 함께 띄운다.
2. `main.cpp`가 조인트 상태를 받은 뒤 현재 자세를 잠시 hold한다.
3. 이후 양팔을 elbow-down ready pose로 천천히 이동시킨다.
4. `/dual_arm/DualArmCmd_sim`에 vision 명령(mode `3`)이 들어오면 목표 지점 표시용 `place_indicator`를 world에 스폰한다.
5. Head는 아래 순서로 스캔한다.
   - yaw `0`
   - pitch `30 deg`
   - 실패 시 `45 deg`
   - 실패 시 `60 deg`
6. 각 스캔 자세에서 `0.5 s` 정지 후 `/aruco_ros/pose`를 본다.
7. 연속 3프레임이 `1 cm` 이내로 일치하면 검출 확정이다.
8. 검출된 마커 pose를 `world`로 TF 변환하고, 마커가 큐브 윗면에 붙어 있다는 가정으로 `z - 0.0505 m` 보정해서 큐브 중심을 계산한다.
9. 그 큐브 중심을 기준으로 양팔 grasp point, standoff point, lift point, transport point를 만든다.
10. 각 Cartesian waypoint를 arm-only IK로 풀어 관절 궤적을 만든다.
11. 제어 루프는 PD 보정 가속도 + Pinocchio `rnea()`로 계산 토크를 만들어 effort controller로 보낸다.
12. `PHASE_GRASP_TO_PLACE` 동안에는 임피던스 항을 추가하고, 동시에 큐브 pose를 양 손끝 중앙으로 계속 업데이트한다.
13. `PHASE_RETURN`으로 넘어갈 때 큐브를 최종 목표 지점으로 한 번 놓고, return 중에도 목표 위치 고정을 잠시 유지한다.
14. 복귀가 끝나면 `/dual_arm/TrajectoryDone`을 발행한다.

## 왜 이번에 비전 인식이 되기 시작했는가

직전 문제는 스캔 로직보다 URDF head pitch 제한이었다.

- 코드에서는 `45 deg`, `60 deg` 재시도를 넣었어도
- `Head_pitch_joint` upper limit가 `0.5236 rad`였기 때문에
- 실제 Gazebo 조인트는 `30 deg` 이상 내려가지 못했다.

이번 수정으로:

- `main.cpp`에 다단계 scan 후보를 추가했고
- `dual_arm.xacro`, `dual_arm.urdf`의 `Head_pitch_joint` upper limit를 `1.0472 rad`로 올렸다.

이후 실제 테스트에서:

- `30 deg` 실패
- `45 deg`에서 marker confirmed
- phase 진행 `3 -> 0 -> 1 -> 2 -> 0`
- 최종 큐브 위치 `x=0.40, y=0.12, z=1.08`

까지 확인했다.

## 관절/IK 구조

모델은 11DoF를 유지한다.

- waist 1
- left arm 4
- right arm 4
- head yaw 1
- head pitch 1

하지만 arm IK는 head와 waist를 쓰지 않는다.

- left arm Pinocchio index: `3..6`
- right arm Pinocchio index: `7..10`

즉, head scan은 head만 움직이고, grasp trajectory는 양팔만 푼다.

## trajectory 구성

vision pick에서 만드는 궤적은 5개 구간이다.

1. current -> standoff
2. standoff -> grasp
3. grasp -> lift
4. lift -> transport
5. transport -> return

의도는 단순하다.

- pedestal과 바로 충돌하지 않게 먼저 옆으로 빠지고
- grasp 직전 접근은 느리게 들어가고
- lift 후에 목표 지점으로 간다.

접촉 직전 구간만 `0.03 m/s`로 낮춰서 충격을 줄인다.

## 제어 구조

기본 토크는 아래 식으로 만든다.

```text
qddot_cmd = trajectory_acc + PD_acc + impedance_acc(optional)
tau = rnea(q, qdot, qddot_cmd)
```

핵심은 RNEA를 다시 살렸다는 점이다.

- 중력
- 코리올리
- 관성항

을 분리 보상하지 않고 `rnea()` 결과를 그대로 쓴다.

## 임피던스와 virtual grasp

현재 코드는 순수 물리 grasp만으로 끝까지 버티는 구조는 아니다.

`PHASE_GRASP_TO_PLACE` 동안:

- F/T 센서 값은 low-pass filter를 거친다.
- 임피던스 항을 관절 가속도 형태로 더한다.
- 동시에 `updateGraspedObjectPose()`가 큐브를 양 손끝 중앙으로 붙여 준다.

즉 현재 데모는:

- arm motion과 contact는 실제로 계산하지만
- object transport 안정화는 Gazebo model pose update 훅에도 의존한다.

또한 `PHASE_RETURN`에서는 `placeObjectAt()`로 큐브를 목표 지점에 고정 배치한다.

따라서 지금 상태를 정확히 표현하면:

- 비전 검출 + 양팔 IK + RNEA effort control + 임피던스 보정
- 그리고 pick/place 성공률을 올리기 위한 virtual grasp / placement hold 보조

의 혼합 구조다.

## 현재 중요한 파라미터

- Head scan pitch candidates: `30, 45, 60 deg`
- Head pitch upper limit: `1.0472 rad`
- Marker to box center offset: `0.0505 m` along `-Z`
- grasp offset: `0.040 m`
- contact approach speed: `0.03 m/s`
- scan settle: `0.5 s`
- scan confirmation: 3 frames
- scan match tolerance: `0.01 m`

## 재현 명령

빌드:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

실행:

```bash
roslaunch dual_arm gazebo.launch gui:=true
```

vision pick 명령:

```bash
rostopic pub -1 /dual_arm/DualArmCmd_sim std_msgs/Float32MultiArray '{data: [3.0, 0.40, 0.12, 1.03]}'
```

## 남아 있는 한계

1. pick-place 성공이 아직 Gazebo model pose update 훅에 부분적으로 의존한다.
2. object orientation은 단순화되어 있고, top marker + axis-aligned spawn 가정을 쓴다.
3. grasp contact 자체만으로 안정 이송되는지에 대한 검증은 아직 별도 단계가 필요하다.
4. 어드미턴스 제어 전환은 아직 안 들어갔다.
