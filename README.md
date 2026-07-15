# Dual Arm Grasp Control

양팔 로봇의 파지(grasp) 및 힘 제어 시뮬레이션 패키지입니다.

## 사용 방법

### 1. 빌드

```bash
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) -l$(nproc)
```

### 2. 실행

**첫 번째 터미널** — Gazebo 시뮬레이션 실행

```bash
roslaunch dual_arm gazebo.launch
```

**두 번째 터미널** — 명령 입력용 CLI 실행

```bash
rosrun dual_arm dual_arm_command
```

### 3. 명령 입력 순서

두 번째 터미널에서 아래 순서대로 입력합니다.

```
modeling
1        # 머리 기울임 + 팔꿈치 90도 위치로 이동 (파지 전 준비 자세)
b        # 이전 화면으로 돌아가기
grasp    # 파지 시퀀스 시작 (접근 → 접촉 → 힘 제어 → 들어올리기 → 회전)
```

## 참고

- `1` 입력 시 이동하는 준비 자세는 `dual_arm_command.cpp`에서 확인/수정 가능합니다.
- `grasp` 입력 이후의 동작은 `main.cpp`의 상태머신(`GraspState`)을 따라 자동으로 진행됩니다.
