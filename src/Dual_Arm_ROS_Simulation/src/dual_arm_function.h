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
const int ARM_DOF = 5;
const int DoF = 13;
// DoF 배열 순서(Pinocchio model.nq 순서와 동일):
// 0:Waist 1:Head_yaw 2:Head_pitch
// 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:L_wy
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
double left_arm_jointp[ARM_DOF] = {0,};
double left_arm_jointv[ARM_DOF] = {0,};
double left_arm_torque[ARM_DOF] = {0,};
double right_arm_jointp[ARM_DOF] = {0,};
double right_arm_jointv[ARM_DOF] = {0,};
double right_arm_torque[ARM_DOF] = {0,};

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

// PD output is an acceleration target passed through RNEA.  Keep the original
// waist/head gains, but use damping-dominant arm gains so the added wrist mass
// does not drive the physical effort limits into sustained saturation.
// Order: Waist, Head_yaw, Head_pitch, L_sp,L_sr,L_sy,L_e,L_wy, R_sp,R_sr,R_sy,R_e,R_wy.
double Kp[DoF] = { 1000, 500, 500, 160, 160, 160, 160, 40, 160, 160, 160, 160, 40 };
double Kd[DoF] = { 50,   30,  30, 25,  25,  25,  25,  8,  25,  25,  25,  25,  8  };

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
int grasp_gate_row = -1;         // lift 세그먼트 진입 직전(마지막 squeeze row 다음) 인덱스
int grasp_gate_end_row = -1;     // return 세그먼트 진입 직전 인덱스; gate는 이 전까지만 유효
bool grasp_contact_ready = false;
bool grasp_acquired_once = false;  // 최초 파지 이후 힘 저하는 궤적 rewind 대신 제자리에서 회복
bool realtime_lift_active = false;
bool realtime_lift_hold_active = false;
int realtime_lift_ticks = 0;
int realtime_lift_hold_ticks = 0;
VectorXd realtime_lift_hold_q = VectorXd::Zero(DoF);
Vector3d realtime_ref_L = Vector3d::Zero();
Vector3d realtime_ref_R = Vector3d::Zero();
const int REALTIME_LIFT_TICKS = 800; // 0.8 s at 1 kHz
const int REALTIME_LIFT_HOLD_TICKS = 2000; // 2 s stabilization before transport
int grasp_contact_ticks = 0;
int grasp_post_contact_hold_ticks = 0;

// F/T 센서 측정값 (force.x,y,z), /dual_arm/left_ft_sensor, /dual_arm/right_ft_sensor 콜백에서 갱신
Vector3d left_ft_force  = Vector3d::Zero();
Vector3d right_ft_force = Vector3d::Zero();
Vector3d left_ft_torque  = Vector3d::Zero();
Vector3d right_ft_torque = Vector3d::Zero();

// F/T 로우패스 필터 상태 (어드미턴스 F_ext로 쓰기 전에 접촉 노이즈 억제용, main.cpp에서 갱신)
Vector3d left_ft_force_lpf     = Vector3d::Zero();
Vector3d right_ft_force_lpf    = Vector3d::Zero();
Vector3d left_ft_force_before  = Vector3d::Zero();
Vector3d right_ft_force_before = Vector3d::Zero();
Vector3d left_ft_torque_lpf     = Vector3d::Zero();
Vector3d right_ft_torque_lpf    = Vector3d::Zero();
Vector3d left_ft_torque_before  = Vector3d::Zero();
Vector3d right_ft_torque_before = Vector3d::Zero();
const double FT_LPF_CUTOFF_HZ  = 10.0;  // 컷오프 주파수 [Hz]
const double GRASP_CONTACT_FORCE_THRESHOLD = 7.5;     // [N] 안정 접촉에서 nominal을 재중심화하는 기준; 제어 목표는 계속 10N
const double GRASP_FACE_CONTACT_TORQUE_THRESHOLD = 0.25; // [N*m] 10N 파지와 넓어진 pad의 정상 모멘트 허용
const int    GRASP_CONTACT_HOLD_TICKS = 120;         // 0.12s @ 1kHz
const int    GRASP_POST_CONTACT_HOLD_TICKS = 500;    // 0.50s @ 1kHz, 접촉 직후 그대로 더 조여서 안정화
const double WRIST_ALIGN_CONTACT_FORCE = 2.0;        // [N] 비접촉 영토크를 정렬 완료로 오인하지 않음
const double WRIST_ALIGN_TORQUE_THRESHOLD = 0.15;    // [N*m] 패드 면 정렬 허용 모멘트
const double WRIST_ALIGN_PRELOAD_FORCE = 5.0;        // [N] 정렬 중 접촉을 잃지 않는 약한 압착력
const int    WRIST_ALIGN_HOLD_TICKS = 200;           // 0.20s @ 1kHz
bool wrist_alignment_ready = false;
int wrist_alignment_ticks = 0;
int grasp_force_loss_ticks = 0;
int left_adm_recenter_count = 0;
int right_adm_recenter_count = 0;
int left_adm_recenter_cooldown = 0;
int right_adm_recenter_cooldown = 0;

// 가상 스프링-댐퍼-질량 파라미터 (튜닝용): Ma*x_ddot + Da*x_dot + Ka*x = F_ext.
// x는 nominal grasp trajectory 위에 덧붙이는 Cartesian compliance offset이다.
double Ma_left[3]      = { 2.0, 2.0, 2.0 };       // 가상 질량 [kg]
double Da_left[3]      = { 65.0, 65.0, 65.0 };    // 가상 댐핑 [N·s/m]
double Ka_left[3]      = { 0.0, 0.0, 0.0 };       // 원래 위치 제어 방식: 가상 위치 스프링 생략

double Ma_right[3]     = { 2.0, 2.0, 2.0 };
double Da_right[3]     = { 65.0, 65.0, 65.0 };
double Ka_right[3]     = { 0.0, 0.0, 0.0 };

Vector3d left_adm_pos  = Vector3d::Zero();
Vector3d left_adm_vel  = Vector3d::Zero();
Vector3d right_adm_pos = Vector3d::Zero();
Vector3d right_adm_vel = Vector3d::Zero();
const double ADMITTANCE_DLS_LAMBDA = 0.05;   // Cartesian offset -> arm joint offset 변환용
const double ADMITTANCE_POS_LIMIT  = 0.05;   // nominal IK가 면에 도달한 뒤 10N까지 압착할 수 있는 변위 여유
const double ADMITTANCE_VEL_LIMIT  = 0.25;   // 축별 최대 순응 속도 [m/s]
const double DESIRED_SQUEEZE_FORCE = 10.0;   // [N] 양쪽 손이 각각 유지할 정상 squeeze 힘
const double GRASP_FORCE_LOSS_THRESHOLD = 7.0; // [N] lift 중 이 값 아래면 재압착
const int GRASP_FORCE_LOSS_TICKS = 100;        // 0.10s @ 1kHz



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

        // === 추가: 댐핑 의사역행렬 (mode 2에서 J -> J+ 변환용) ===
        MatrixXd DampedPinv(const MatrixXd& J, double lambda);     

};

DualArmControl dualarm; //class의 변수설정
