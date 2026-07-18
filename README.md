# 실행 방법

## 브랜치 안내

이 저장소는 서로 다른 데모를 담은 두 브랜치를 유지합니다. 두 브랜치는 서로 덮어쓰지 않는
독립된 실험이라, 원하는 쪽으로 자유롭게 전환해서 쓰면 됩니다.

| 브랜치 | 내용 |
|---|---|
| `admittance-control-table-viz` | 테이블 위 물체를 인식(ArUco) → 파지 → 이송 → 내려놓기 (기본 pick & place 데모) |
| `vertical-lift-experiment` (현재 브랜치) | 물체를 제자리(y=0)에서 파지 → 팔이 안정적으로 도달 가능한 최대 높이(z≈1.5m)까지 수직으로 들어올렸다가 같은 자리에 다시 내려놓기 (F/T 센서 수직 지지력 확인용, 물체 질량 1.6kg) |

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

물체(aruco_box_26) 초기 위치: `x=0.45, y=0, z=1.075` (world frame, meter), 질량 1.6kg —
`worlds/rrbot.world` / `models/aruco_box_26/model.sdf`에 고정.

Vision pick & 수직 리프트 실행:

```
vision       ← vision 모드 선택
0            ← 이송 목표 좌표 입력 선택
0.45 0 1.075   ← 아무 값이나 입력 (이 브랜치에서는 이송 목표를 쓰지 않고, 파지 지점
                 자체로 다시 돌아오므로 값은 무시됨 - 형식만 맞추면 됨)
```

물체를 파지한 뒤 같은 x,y를 유지한 채 z≈1.5m까지 수직 상승 → 다시 같은 x,y로 z=1.075까지
하강 → 놓기 → 후퇴 → 복귀 순서로 동작합니다.

**rqt_plot으로 F/T 확인** (양팔 F/T 센서, 로컬 프레임 기준):

```
/dual_arm/left_ft_sensor/wrench/force/x
/dual_arm/left_ft_sensor/wrench/force/y   ← 스퀴즈(압착)력, 목표 10N 고정
/dual_arm/left_ft_sensor/wrench/force/z
/dual_arm/right_ft_sensor/wrench/force/x
/dual_arm/right_ft_sensor/wrench/force/y  ← 스퀴즈(압착)력, 목표 10N 고정
/dual_arm/right_ft_sensor/wrench/force/z
```

`force/y`는 그립패드 법선이 항상 EE 로컬 ±Y축이라 자세와 무관하게 스퀴즈력을 나타냅니다
(어드미턴스 제어 목표 10N). 수직 지지력(무게를 받치는 힘)은 팔 자세가 많이 바뀌는 이
실험 특성상 고정된 축이 없어 `force/x`, `force/z`에 섞여 나타나므로 둘을 같이 보면서
리프트 구간(공중에서 무게를 손으로만 받치는 구간)의 크기를 확인하면 됩니다.

## 종료

```bash
killall -9 gzclient gzserver roslaunch rosmaster
```
