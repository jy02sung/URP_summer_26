# Gazebo Manual Run Guide

이 문서는 `~/catkin_ws`에서 현재 dual arm 시뮬레이션을 수동으로 실행하고 확인할 때 쓰는 최소 절차를 정리한 것이다.

## 1. 작업 폴더로 이동

```bash
cd ~/catkin_ws
```

## 2. 빌드

```bash
catkin_make
source devel/setup.bash
```

빌드 후 새 터미널을 열었으면 다시 아래를 해줘야 한다.

```bash
cd ~/catkin_ws
source devel/setup.bash
```

## 3. 기존 ROS/Gazebo 프로세스 정리

과거 세션이 남아 있으면 Gazebo 중복 스폰, controller 충돌, `/clock` 이상이 생길 수 있다.

```bash
pkill -f "roslaunch dual_arm gazebo.launch|gzserver|gzclient|dual_arm_main|controller_manager/spawner|robot_state_publisher|aruco_ros/single|static_transform_publisher|roscore|rosmaster" || true
```

정리 확인:

```bash
pgrep -af "roslaunch|gzserver|gzclient|dual_arm_main|controller_manager/spawner|robot_state_publisher|aruco_ros/single|static_transform_publisher|roscore|rosmaster"
```

아무것도 안 나오면 깨끗한 상태다.

## 4. Gazebo 실행

GUI 포함 실행:

```bash
roslaunch dual_arm gazebo.launch gui:=true
```

GUI 없이 실행:

```bash
roslaunch dual_arm gazebo.launch gui:=false
```

## 5. 정상 기동 체크 포인트

런치 로그에서 아래 문구가 보여야 한다.

```text
Pinocchio joint order verified: Waist, Head_yaw, Head_pitch, L_sp, L_sr, L_sy, L_e, R_sp, R_sr, R_sy, R_e
```

초기 조건:

- 양쪽 elbow는 시작 시 약 `1.0 rad` 굽힘
- table, cube, aruco detector가 같이 올라와야 함
- `main` 노드가 같이 떠야 함

## 6. 관절 상태 확인

```bash
rostopic echo -n 1 /dual_arm/joint_states
```

마커 pose 확인:

```bash
rostopic echo -n 1 /aruco_single/pose
```

## 7. Pick 동작 수동 트리거

문자열 명령으로 실행:

```bash
rostopic pub -1 /dual_arm/command std_msgs/String "data: 'ArucoPickCmd'"
```

또는 Empty 명령으로 실행:

```bash
rostopic pub -1 /dual_arm/ArucoPickCmd std_msgs/Empty "{}"
```

## 8. 로그에서 볼 상태 전이

정상적으로 시작되면 보통 아래 순서로 로그가 나온다.

```text
[Scan] Moving head to scan pose.
[Scan] Head reached scan pose. Settling.
[Scan] Checking stable ArUco detection.
[Scan] Marker confirmed.
[Pick] Precomputed dual-arm pipeline built.
```

## 9. 종료

`roslaunch`를 실행한 터미널에서 `Ctrl+C`

남은 프로세스가 있으면 마지막으로 정리:

```bash
pkill -f "roslaunch dual_arm gazebo.launch|gzserver|gzclient|dual_arm_main|controller_manager/spawner|robot_state_publisher|aruco_ros/single|static_transform_publisher|roscore|rosmaster" || true
```

## 10. 자주 쓰는 디버깅 명령

현재 토픽 목록:

```bash
rostopic list
```

clock 확인:

```bash
rostopic echo -n 1 /clock
```

main 노드 로그 확인:

```bash
rosnode list
rosnode info /main
```

프로세스 확인:

```bash
pgrep -af "gzserver|gzclient|dual_arm_main|roslaunch dual_arm gazebo.launch"
```
