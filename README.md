# 실행 방법

## 사전 준비

```bash
cd ~/friend_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 터미널 1 — ROS 마스터

```bash
roscore
```

## 터미널 2 — Gazebo + 로봇 스폰 + 제어 노드

```bash
roslaunch dual_arm gazebo.launch
```

## 터미널 3 — ArUco 인식 노드

```bash
roslaunch dual_arm aruco_detection.launch
```

## 터미널 4 — 명령 입력

```bash
rosrun dual_arm dual_arm_command
```

물체(aruco_box_26) 초기 위치: `x=0.45, y=-0.23, z=1.075` (world frame, meter) — `worlds/rrbot.world`에 고정 스폰.

Vision pick & place 실행:

```
vision       ← vision 모드 선택
0            ← 이송 목표 좌표 입력 선택
0.45 0.3 1.1   ← 이송 목표 [x y z] (world frame, meter)
```

## 종료

```bash
killall -9 gzclient gzserver roslaunch rosmaster
```
