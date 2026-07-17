#include "pinocchio/parsers/urdf.hpp"          // URDF 파일을 파싱하여 모델 생성
#include "pinocchio/algorithm/joint-configuration.hpp" // randomConfiguration 등 제공
#include "pinocchio/algorithm/kinematics.hpp"   // forwardKinematics 제공
#include "pinocchio/algorithm/frames.hpp"       // 프레임 업데이트를 위한 함수 제공
#include "pinocchio/algorithm/jacobian.hpp"     // 자코비안 계산 함수 제공
#include <pinocchio/algorithm/rnea.hpp>         // 동역학 정보 제공

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <csignal>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <algorithm>  

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace std;
using namespace Eigen;

#define POSITION 1
#define EFFORT 2
#define ARMCTRLMODE 2

// 전역 변수 선언
const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;
const int DoF = 13;
// DoF 배열 순서(Pinocchio model.nq 순서와 동일해야 함, urdf 트리 순회 결과로 실측 확인됨,
// Task 1 Step 5에서 pinocchio 로드로 재검증):
// 0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:L_wy
// 8:R_sp 9:R_sr 10:R_sy 11:R_e 12:R_wy
const double SAMPLING_TIME = 0.001;
const double SAMPLING_TIME_TRAJ = 0.001;

int command_mode = 0;   // 0:modeling(관절각) 1:joint sim(IK+5차) 2:cartesian sim(직선)
double dual_arm_commandx[6] = {0,};   // 목표 EE 위치 [Lx,Ly,Lz,Rx,Ry,Rz]
MatrixXd dual_arm_cart_pos_trajectory = MatrixXd::Zero(1,6);  // 직선 궤적(직교)
MatrixXd dual_arm_cart_vel_trajectory = MatrixXd::Zero(1,6);  // 직교 속도
MatrixXd dual_arm_cart_acc_trajectory = MatrixXd::Zero(1,6);  // 직교 가속도
VectorXd q_ik_seed = VectorXd::Zero(DoF);   // IK 시드
VectorXd q_ik_result = VectorXd::Zero(DoF); // IK 결과

// vision pick(mode 3) 세그먼트 빌드 시 사전 계획된 desired Cartesian trajectory(x_d, ẋ_d, ẍ_d).
// dual_arm_jointp_trajectory와 행(row) 인덱스가 1:1로 대응 - PHASE_GRASP_TO_PLACE 구간에서
// 어드미턴스가 매 tick 이 값을 조회한다 (그 외 구간은 채워지긴 하지만 조회되지 않음).
// 열 순서는 dual_arm_cart_pos_trajectory와 동일 [Lx,Ly,Lz,Rx,Ry,Rz].
MatrixXd dual_arm_cart_target_trajectory     = MatrixXd::Zero(1,6);  // x_d
MatrixXd dual_arm_cart_target_vel_trajectory = MatrixXd::Zero(1,6);  // ẋ_d
MatrixXd dual_arm_cart_target_acc_trajectory = MatrixXd::Zero(1,6);  // ẍ_d

bool callback = false;
int traj_cnt = 1;
bool traj_done_published = false;   // 현재 궤적의 TrajectoryDone 발행 여부 (콜백마다 리셋)
VectorXi dual_arm_phase_trajectory = VectorXi::Zero(1);  // 각 궤적 행(row)이 속한 TaskPhase (vision pick 세그먼트 태깅용)

// 각 관절 상태 저장 (필요 시 콜백 함수에서 사용)
double waist_jointp[1] = {0};
double waist_jointv[1] = {0};
double waist_torque[1] = {0};
double head_jointp[2] = {0,};   // [0]=yaw, [1]=pitch
double head_jointv[2] = {0,};
double head_torque[2] = {0,};
double left_arm_jointp[4] = {0,};
double left_arm_jointv[4] = {0,};
double left_arm_torque[4] = {0,};
double right_arm_jointp[4] = {0,};
double right_arm_jointv[4] = {0,};
double right_arm_torque[4] = {0,};

double dual_arm_jointp[DoF] = {0,};
double dual_arm_jointv[DoF] = {0,};
VectorXd dual_arm_jointp_vec = VectorXd::Zero(DoF);
VectorXd dual_arm_jointv_vec = VectorXd::Zero(DoF);

double dual_arm_jointv_lpf[DoF] = {0,};
VectorXd dual_arm_jointv_lpf_vec = VectorXd::Zero(DoF);
double dual_arm_jointv_before[DoF] = {0,};

double dual_arm_initp[DoF] = {0,};
double dual_arm_commandp[DoF] = {0,};

double dual_arm_targetp[DoF] = {0,};
double dual_arm_targetv[DoF] = {0,};
VectorXd dual_arm_targetp_vec = VectorXd::Zero(DoF);
VectorXd dual_arm_targeta_vec = VectorXd::Zero(DoF);

MatrixXd dual_arm_jointp_trajectory = MatrixXd::Zero(1,DoF); 
MatrixXd dual_arm_jointv_trajectory = MatrixXd::Zero(1,DoF);
MatrixXd dual_arm_jointa_trajectory = MatrixXd::Zero(1,DoF);

// double Kp[DoF] = { 1000, 500, 300, 50, 100, 500, 300, 50, 100 };
// double Kd[DoF] = { 10, 3, 3, 1, 1, 3, 3, 1, 1 };

// double Kp[DoF] = { 1000, 500, 350, 50, 100, 500, 350, 50, 100 }; //이거 사용
// double Kd[DoF] = { 10, 3, 1.5, 0.5, 1, 3, 1.5, 0.5, 1 };

// 순서: Waist, Head_yaw, Head_pitch, L_sp,L_sr,L_sy,L_e,L_wy, R_sp,R_sr,R_sy,R_e,R_wy
// head 게인은 초기값(관성이 작아 팔보다 낮게 시작) - 실제 거동 보고 재튜닝 필요
// 손목 yaw(인덱스 7, 12)는 접촉 컴플라이언스용 소프트 게인(Task 2 yaml PID p:40/d:8과 동일)
double Kp[DoF] = { 1000, 500, 500, 500, 500, 500, 500, 40, 500, 500, 500, 500, 40 };
double Kd[DoF] = { 50,   30,  30,  1,   1,   1,   1,   8,  1,   1,   1,   1,   8  };

double PD_torque[DoF] = {0, };
double PD_acc[DoF] = {0, };

VectorXd gravity_torque = VectorXd::Zero(DoF);
VectorXd nonlinear_torque = VectorXd::Zero(DoF);
VectorXd dynamic_torque = VectorXd::Zero(DoF);

double target_torque[DoF] = {0, };

////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------- Admittance Control --------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// 작업 단계: 스캔 / 접근 / 파지~내려놓기 / 복귀. 파지~내려놓기 구간에서만 어드미턴스 활성화.
// vision pick(command_mode==3) 실행 시 main.cpp가 세그먼트별로 dual_arm_phase_trajectory에
// 태깅해서 재생 중 자동으로 전환한다. 그 외 모드(0/1/2) 또는 idle 상태에서는
// /dual_arm/TaskPhase(std_msgs/Int32) 구독으로 수동 오버라이드 가능 (기본값: 접근, 어드미턴스 OFF).
// PHASE_SCAN(3)은 Head 자동 스캔 중에만 내부적으로 쓰이며 수동 오버라이드 대상이 아니다
// (msgCallbackTaskPhase의 범위 체크가 PHASE_APPROACH~PHASE_RETURN까지만 허용).
enum TaskPhase { PHASE_APPROACH = 0, PHASE_GRASP_TO_PLACE = 1, PHASE_RETURN = 2, PHASE_SCAN = 3 };
int task_phase = PHASE_APPROACH;

// F/T 센서 측정값 (force.x,y,z), /dual_arm/left_ft_sensor, /dual_arm/right_ft_sensor 콜백에서 갱신
Vector3d left_ft_force  = Vector3d::Zero();
Vector3d right_ft_force = Vector3d::Zero();

// F/T 로우패스 필터 상태 (어드미턴스 F_ext로 쓰기 전에 접촉 노이즈 억제용, main.cpp 어드미턴스 블록에서 갱신)
Vector3d left_ft_force_lpf     = Vector3d::Zero();
Vector3d right_ft_force_lpf    = Vector3d::Zero();
Vector3d left_ft_force_before  = Vector3d::Zero();
Vector3d right_ft_force_before = Vector3d::Zero();
const double FT_LPF_CUTOFF_HZ  = 10.0;  // 컷오프 주파수 [Hz]

// 가상 질량-댐핑-강성 파라미터 (튜닝용). 배열 인덱스는 world frame [x,y,z]지만, 이 로봇의 실제
// 스퀴즈(파지) 방향은 world Z(수직)가 아니라 world Y다 - objL/objR이 obj ± (0,grasp_offset,0)로
// y축 양쪽에서 마주보고 조이는 구조이기 때문 (main.cpp buildGraspPipelineFromDetection 참고).
// 그래서 힘추종은 인덱스 1(y)에, 위치추종은 인덱스 0,2(x,z - z는 들어올리기/이송 높이)에 적용한다.
// x,z(인덱스 0,2): 위치추종 admittance - M(ẍcmd−ẍd) + D(ẋcmd−ẋd) + K(xcmd−xd) = 0
// y(인덱스 1):     힘추종 admittance   - M·ÿcmd + D·ẏcmd = F_ext,y − F_d,y  (K_y=0 고정, 아래서 0으로 둠)
double Ma_left[3] = { 2.0, 2.0, 2.0 };      // 가상 질량 [kg]
double Da_left[3] = { 65.0, 65.0, 65.0 };   // 가상 댐핑 [N·s/m]
double Ka_left[3] = { 500.0, 0.0, 500.0 };  // 가상 강성 [N/m] - y(스퀴즈)는 순수 힘제어라 0 고정

double Ma_right[3] = { 2.0, 2.0, 2.0 };
double Da_right[3] = { 65.0, 65.0, 65.0 };
double Ka_right[3] = { 500.0, 0.0, 500.0 };

// 스퀴즈 목표 힘 F_d,y [N]. 왼팔은 obj +y쪽에서 -y로 누르고 오른팔은 obj -y쪽에서 +y로 눌러
// 서로를 향해 조이므로, F/T가 world frame으로 변환된 뒤의 부호는 팔마다 반대다 - 2026-07-13 Gazebo
// 실측(PHASE_APPROACH 구간, 어드미턴스 미개입 순수 위치유지 스퀴즈)으로 확인:
// F_ext_L(y) 평균 +5.4N(양수), F_ext_R(y) 평균 -3.4N(음수). 그래서 목표값도 팔마다 부호를 맞춘다.
const double ADMITTANCE_FD_Y_LEFT  =  10.0;
const double ADMITTANCE_FD_Y_RIGHT = -10.0;

// 어드미턴스 command 적분 상태 (tick 간 유지). PHASE_GRASP_TO_PLACE 진입 순간 실제 EE 위치로
// 초기화되고(admittance_initialized), 그 밖에서는 다음 진입에 대비해 리셋된다 (main.cpp).
Vector3d xL_cmd     = Vector3d::Zero();
Vector3d xL_cmd_dot = Vector3d::Zero();
Vector3d xR_cmd     = Vector3d::Zero();
Vector3d xR_cmd_dot = Vector3d::Zero();
bool admittance_initialized = false;

// y(스퀴즈)를 "기준 궤적 없는 순수 힘추종"으로 처음 구현했더니(2026-07-13 1차 Gazebo 검증) 물체가
// 거의 옮겨지지 않았다 - 이 로봇은 이송목표가 pick 지점 대비 주로 Y방향으로 떨어져 있어(objL.y≈-0.11
// -> transportL.y≈0.44, 약 0.55m) 스퀴즈 축(Y)이 동시에 이송 방향이기도 하기 때문. PRD 수식이 z(스퀴즈)에
// 기준궤적 항을 안 둔 건 "스퀴즈 축 ⊥ 이송 방향"을 암묵 전제한 것인데 이 로봇 기하에서는 안 맞았다.
// 그래서 y_cmd = y_d(t)(계획된 이송 경로, 기준) + deltaY(힘오차로 만든 순응 변위)로 재정의한다 -
// 이송은 y_d(t)가 그대로 담당하고, deltaY만 K=0 힘추종 법칙(M*deltaY_ddot + D*deltaY_dot = F_ext,y - F_d,y)을
// 그대로 따른다. deltaY_dot/deltaY도 접촉 순간유실 시 무한정 커지는 것을 막기 위해 속도/변위를 하드 리밋한다
// (스프링이 아니라 리밋 - 1차 검증에서 리밋 없이는 실제로 발산해 물체가 바닥에 떨어지는 것을 확인).
double deltaYL = 0.0, deltaYR = 0.0;   // y_cmd의 y_d(t) 대비 순응 변위 (PHASE_GRASP_TO_PLACE 진입 시 현재 오프셋으로 초기화)
// 2026-07-13 2차 검증(deltaY 도입 후): 물체는 실제로 옮겨지기 시작했지만 fy가 목표(±10N) 근처에
// 못 미친 채(수~기N대) deltaY가 ±15mm에서 막혀 스퀴즈력 부족 -> 이송 중 관성부하를 못 버티고
// 슬립/낙하. 15mm는 접촉면을 충분히 눌러 10N을 낼 만큼 깊지 않았던 것으로 판단, 30mm로 확대.
// 속도 리밋(0.03m/s)은 그대로 유지 - 발산 방지는 변위가 아니라 속도 쪽이 핵심이었음(1차 검증).
const double Y_CMD_MAX_DISP = 0.03;   // y_d(t) 기준 deltaY 최대 변위 [m]
const double Y_CMD_VEL_LIMIT = 0.03;  // deltaY 최대 속도 [m/s] (기존 APPROACH_CONTACT_V_DES와 동일한 완만한 접촉 속도)

// 매 tick IK 웜스타트 + 관절 속도/가속도 후진차분용 (온라인 계산이라 중심차분 대신 후진차분 사용)
VectorXd q_cmd_prev     = VectorXd::Zero(DoF);
VectorXd q_cmd_dot_prev = VectorXd::Zero(DoF);



// SE3 T_world_A = data.oMf[A];   // 또는 oMi[A]도 가능
// SE3 T_world_B = data.oMf[B];

// SE3 T_A_B = T_world_A.inverse() * T_world_B; 기준프레임을 world에서 A로 변환하기 (A프레임 기준으로 본B의 pose)


class DualArmControl
{
    private:
       

    public:
        DualArmControl();
        ~DualArmControl();

        double LowPassFilter(double &input, double &output_before, double cutoff_frequency);
        void JointTrajectoryTrapezoidal(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out);
        void JointTrajectoryQuintic(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out, MatrixXd& q_acc_out);
        void PDController(double* target_q, double* current_q, double* target_q_dot, double* current_q_dot, double* PDtorque);
        
        // === 추가: DLS 역기구학 (위치만, 좌우 동시) ===
        // 목표 좌우 EE 위치를 받아 9개 목표 관절각을 q_out 에 채운다.
        // model, data, 프레임 ID는 main에서 넘겨받음. q_seed는 IK 초기 추정값(보통 현재 관절각).
        void SolveIK_Position(pinocchio::Model& model, pinocchio::Data& data,
                              pinocchio::FrameIndex l_EE, pinocchio::FrameIndex r_EE,
                              const Vector3d& target_L, const Vector3d& target_R,
                              const VectorXd& q_seed, VectorXd& q_out);

        // === 추가: 직교 공간 직선 궤적 생성 ===
        // 시작 EE 위치(start)에서 목표 EE 위치(goal)까지 5차 시간 스케일링으로 직선 보간.
        // 좌우 각각 x,y,z 경로를 만든다.
        void CartesianLineTrajectory(const Vector3d& startL, const Vector3d& goalL,
                             const Vector3d& startR, const Vector3d& goalR,
                             MatrixXd& pos_out, MatrixXd& vel_out, MatrixXd& acc_out,
                             double v_des = 0.1);


};

DualArmControl dualarm; //class의 변수설정