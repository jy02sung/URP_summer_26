# `catkin_ws` vs `catkin_ws_jmp` 제어 로직 비교

## 비교 대상

- 현재 작업 중인 코드: `/home/jys/catkin_ws/src/Dual_Arm_ROS_Simulation`
- 비교 기준 코드: `/home/jys/catkin_ws_jmp/src/urp_summer_26`

`catkin_ws_jmp`는 `origin/jmp` 최신 커밋 `d1e806b`까지 pull했다.  
다만 pull 과정에서 기존 로컬 수정(`CMakeLists.txt`, `src/main.cpp`)과 충돌했고, 그 변경은 `stash@{0}`에 안전하게 남아 있다.

## 한 줄 결론

`jmp` 쪽이 덜 떨리고 더 잘 가는 이유는 단순히 "RNEA를 쓴다"가 아니라,  
`RNEA에 넣기 전의 target q / qdot / qddot을 더 일관되게 만들고`,  
`접촉 구간만 별도 phase로 분리해서 임피던스를 제한적으로 켜고`,  
`양팔 IK를 매 tick 독립적으로 비틀지 않고 미리 세그먼트 궤적으로 구성하기 때문`이다.

반대로 현재 코드는 startup 보정, head hold, Cartesian 매 tick IK, grasp squeeze, hold가 한 루프 안에 섞여 있고,  
RNEA 입력 가속도도 대부분 `PD_acc` 중심이라 전신이 쉽게 흔들린다.

## 가장 큰 차이

### 1. `jmp`는 "궤적을 먼저 만들고", 현재 코드는 "루프에서 즉석으로 많이 결정한다"

`jmp`:
- command가 들어오면 먼저 전체 관절 궤적을 만든다.
- joint sim / cartesian sim / vision pick 모두 `dual_arm_jointp_trajectory`, `dual_arm_jointv_trajectory`, `dual_arm_jointa_trajectory`를 생성한 뒤 재생한다.
- 근거:
  - `src/main.cpp:597-683`
  - `src/main.cpp:713-733`

현재 코드:
- startup도 루프 안에서 그때그때 `lift -> forward`를 생성한다.
- vision pick도 `pick_step`에 따라 매 tick IK를 다시 풀고, hold와 head 고정도 중간에 덮어쓴다.
- 근거:
  - `src/main.cpp:363-405`
  - `src/main.cpp:407-598`
  - `src/main.cpp:604-647`

의미:
- `jmp`는 RNEA가 "이미 매끈한 q/qdot/qddot"를 따라가게 한다.
- 현재 코드는 RNEA가 "매 tick 바뀌는 목표와 모순된 hold/IK 결과"를 동시에 받는다.
- 이 차이가 전신 떨림에 직접적이다.

### 2. `jmp`는 양팔을 하나의 동시 IK 문제로 푼다

`jmp`:
- `SolveIK_Position()`이 좌우 EE를 6차원 스택 태스크로 동시에 푼다.
- 자코비안도 `J = [JL; JR]`로 묶어서 한 번에 해를 구한다.
- 근거:
  - `src/dual_arm_function.cpp:204-257`
  - `src/dual_arm_function.h:168-185`

현재 코드:
- 왼팔 IK를 먼저 풀고, 그 결과 `q_ik`에 이어서 오른팔 IK를 다시 푼다.
- 즉 좌우가 결합된 하나의 문제라기보다, 같은 상태 벡터를 순차 갱신하는 구조다.
- 근거:
  - `src/main.cpp:498-502`
  - `src/main.cpp:612-617`

의미:
- 현재 방식은 왼팔을 맞춘 다음 오른팔이 다시 전체 자세를 비트는 식이어서, waist/shoulder/elbow가 흔들릴 여지가 크다.
- `jmp`는 처음부터 좌우를 동시에 만족하는 쪽으로 간다.

### 3. `jmp`는 contact-sensitive 구간만 임피던스를 켠다

`jmp`:
- `TaskPhase`를 두고 `PHASE_GRASP_TO_PLACE`에서만 임피던스를 활성화한다.
- F/T 센서는 low-pass 후 world frame으로 변환해서 virtual mass-damper-spring 식으로 반영한다.
- 이 가속도를 `dual_arm_targeta_vec`에 더하고, RNEA는 그 가속도까지 포함해 토크를 만든다.
- 근거:
  - `src/dual_arm_function.h:112-145`
  - `src/main.cpp:769-845`

현재 코드:
- 접촉 센서는 squeeze 종료 조건 정도로만 쓰고 있다.
- 힘 센서 기반 임피던스/어드미턴스가 없다.
- 대신 pick 단계에서 contact가 안 오면 더 조이는 식이다.
- 근거:
  - `src/main.cpp:479-527`

의미:
- `jmp`는 물체를 잡은 뒤에는 "정확히 목표 좌표를 강제로 유지"하지 않고, 접촉 힘에 따라 약간 순응한다.
- 현재 코드는 접촉 후에도 거의 동일한 강체식 추종이라 물체/팔/몸통이 서로 밀면서 떨기 쉽다.

### 4. `jmp`는 접근 경로를 충돌 회피 기준으로 분해해놨다

`jmp`:
- 물체 옆 `standoff`로 먼저 이동
- 그다음 낮은 속도로 접촉 접근
- 그다음 수직으로 lift
- 그다음 운반
- 근거:
  - `src/main.cpp:364-447`

현재 코드:
- ArUco로 얻은 큐브 위치에 대해 바로 양팔 target을 만들고,
- squeeze, lift, place를 상대적으로 단순한 단계 전환으로 처리한다.
- startup도 "일단 위로, 앞으로"라는 보정 시퀀스가 별도로 추가됐다.
- 근거:
  - `src/main.cpp:435-465`
  - `src/main.cpp:529-587`

의미:
- `jmp`는 경로 설계가 먼저다.
- 현재 코드는 "컨트롤로 버텨보는" 성격이 더 강하다.

### 5. `jmp`는 joint order가 Pinocchio 순서와 거의 동일하게 유지된다

`jmp`:
- 내부 DoF 배열 순서를 Pinocchio model 순서와 동일하게 둔다.
- 근거:
  - `src/dual_arm_function.h:38-40`

현재 코드:
- ROS controller 순서와 Pinocchio 순서가 달라서 `ctrl_to_pin`, `pin_to_ctrl`로 계속 remap한다.
- 근거:
  - `src/main.cpp:131-170`
  - `src/main.cpp:350-354`
  - `src/main.cpp:684-690`

의미:
- 현재 코드가 틀렸다는 뜻은 아니다.
- 다만 매 단계에서 remap이 섞이니 startup, hold, IK, RNEA 중 어디 한 군데라도 인덱스를 잘못 다루면 바로 이상 자세가 난다.
- `jmp`는 그 위험이 훨씬 작다.

### 6. 현재 코드는 startup/hold가 본 제어에 섞여 있다

현재 코드:
- startup Cartesian sequence
- home capture
- head-only hold
- ArUco wait hold
- Cartesian 매 tick IK
- squeeze loop
- RNEA torque

가 모두 한 루프 안에서 분기된다.
- 근거:
  - `src/main.cpp:337-405`
  - `src/main.cpp:407-598`
  - `src/main.cpp:653-708`

`jmp`:
- startup 해킹이 거의 없고,
- 명령이 들어오면 trajectory를 만든 뒤 replay하는 구조다.
- scan도 별도 상태머신이지만 최종적으로는 trajectory 파이프라인으로 귀결된다.
- 근거:
  - `src/main.cpp:290-310`
  - `src/main.cpp:477-515`
  - `src/main.cpp:597-733`

의미:
- 현재 코드는 "안정화용 예외 로직"이 많아질수록 전체 제어가 더 복잡해진다.
- `jmp`는 예외도 결국 궤적 생성 쪽으로 수렴시켜서 구조가 덜 흔들린다.

## RNEA 자체는 누구 쪽이 더 낫나

RNEA 사용 자체만 보면 `jmp` 쪽이 더 정석에 가깝다.

`jmp`:
- `dual_arm_targeta_vec = planned_qddot + PD_acc`
- 필요 시 여기에 임피던스에서 나온 추가 가속도를 더함
- 마지막에 `rnea(q, qdot, qddot_target)` 호출
- 근거:
  - `src/main.cpp:720-724`
  - `src/main.cpp:785-845`
  - `src/main.cpp:848-856`

현재 코드:
- 대부분의 경우 `dual_arm_targeta_vec`가 사실상 `PD_acc` 중심이다.
- Cartesian 단계도 qddot를 trajectory에서 직접 만들지 않고, target position을 다시 PD로 밀어 넣는 비중이 크다.
- 근거:
  - `src/main.cpp:643-646`
  - `src/main.cpp:668-670`
  - `src/main.cpp:682-690`

의미:
- `jmp`는 RNEA에 "의미 있는 목표 가속도"를 넣는다.
- 현재 코드는 RNEA가 사실상 강한 pose hold를 대신하는 경우가 많다.

## 왜 `jmp` 쪽은 몸이 덜 떨리는가

실질적인 이유를 정리하면:

1. trajectory가 미리 만들어져서 `q`, `qdot`, `qddot`가 연속적이다.
2. 양팔 IK를 동시에 풀어 좌우가 서로 posture를 깨는 현상이 적다.
3. contact 구간만 임피던스를 켜서 rigid tracking을 계속하지 않는다.
4. 접근 경로를 standoff / slow contact / lift / carry로 나눠 충돌과 충격을 줄였다.
5. Pinocchio 순서와 내부 배열 순서를 맞춰서 remap 리스크가 적다.
6. startup/hold 예외 로직이 적어, 본 제어 루프가 단순하다.

## 지금 코드에서 더 덧대는 것보다, 따라가야 할 우선순위

내 판단으로는 아래 순서가 맞다.

1. `jmp`의 내부 DoF 순서 구조를 그대로 가져오기
2. 양팔 동시 IK(`SolveIK_Position`) 구조로 바꾸기
3. "명령이 들어오면 trajectory 전체를 먼저 구성"하는 방식으로 바꾸기
4. current repo의 `pick_step` 즉석 IK/hold 구조를 줄이고, segment replay 방식으로 통합하기
5. F/T low-pass + phase-limited impedance를 grasp-to-place 구간에만 추가하기
6. startup 보정 시퀀스는 마지막에 다시 얹기

즉, 지금 repo에 patch를 더 덧대서 해결하기보다,  
`jmp`의 "trajectory-first + dual-arm simultaneous IK + phase-based impedance" 뼈대를 이식하는 편이 훨씬 낫다.

## 바로 가져오면 좋은 파일/로직

우선순위 높은 순서:

1. `catkin_ws_jmp/src/urp_summer_26/src/main.cpp`
   - `command_mode`
   - `buildGraspPipelineFromDetection()`
   - `addCartesianSegment()`
   - `TaskPhase` / `TrajectoryDone`
   - impedance block

2. `catkin_ws_jmp/src/urp_summer_26/src/dual_arm_function.cpp`
   - `SolveIK_Position()`
   - `CartesianLineTrajectory()`
   - `JointTrajectoryQuintic()`의 좌우 독립 Tf 계산

3. `catkin_ws_jmp/src/urp_summer_26/src/dual_arm_function.h`
   - Pinocchio 순서 기준 DoF 정의
   - impedance/FT 관련 상태 정의

## 내 권장 결론

현재 `catkin_ws` 코드는 "RNEA를 살리기 위해 예외 로직을 계속 덧댄 상태"에 가깝다.  
반면 `jmp` 코드는 "trajectory generator + phase manager + limited impedance + RNEA executor"로 분리가 되어 있다.

그래서 다음 작업은 현재 코드 미세튜닝보다:

- `jmp`의 DoF 순서
- `jmp`의 trajectory 생성/재생 구조
- `jmp`의 dual-arm simultaneous IK
- `jmp`의 phase-based impedance

이 네 축을 현재 repo로 가져오는 방식이 맞다.
