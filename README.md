# 실행 방법

## 브랜치 안내

이 저장소는 서로 다른 데모를 담은 두 브랜치를 유지합니다. 두 브랜치는 서로 덮어쓰지 않는
독립된 실험이라, 원하는 쪽으로 자유롭게 전환해서 쓰면 됩니다.

| 브랜치 | 내용 |
|---|---|
| `admittance-control-table-viz` (현재 브랜치) | 테이블 위 물체를 인식(ArUco) → 파지 → 이송 → 내려놓기 (기본 pick & place 데모) |
| `vertical-lift-experiment` | 물체를 제자리(y=0)에서 파지 → 팔이 안정적으로 도달 가능한 최대 높이(z≈1.5m)까지 수직으로 들어올렸다가 같은 자리에 다시 내려놓기 (F/T 센서 수직 지지력 확인용, 물체 질량 1.6kg) |

### 브랜치 전환

```bash
git checkout admittance-control-table-viz   # 기본 pick & place 데모로
git checkout vertical-lift-experiment       # 수직 리프트 실험으로
```

**주의**: 브랜치를 바꾸면 `src/main.cpp`와 `worlds/rrbot.world`(물체 초기 위치·질량 등)가
달라집니다. 전환할 때마다 아래 순서를 지켜야 합니다.

1. 재빌드 (안 하면 이전 브랜치의 실행 파일이 그대로 남아있음):
   ```bash
   source /home/jungmin/friend_ws_88pct/devel/setup.bash
   catkin_make --only-pkg-with-deps dual_arm
   ```
2. Gazebo가 이미 떠 있다면 완전히 종료 후 재시작 (재빌드만으로는 world 파일 변경 — 물체
   위치/질량 — 이 반영되지 않고, 이전 world 상태 그대로 유지됨):
   ```bash
   killall -9 gzclient gzserver roslaunch rosmaster
   ```
   그 다음 아래 "터미널 1~4"를 처음부터 다시 실행.

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
