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
const int DoF = 11;
// DoF 배열 순서(Pinocchio model.nq 순서와 동일해야 함, urdf 트리 순회 결과로 실측 확인됨):
// 0:Waist 1:Head_yaw 2:Head_pitch 3:L_sp 4:L_sr 5:L_sy 6:L_e 7:R_sp 8:R_sr 9:R_sy 10:R_e
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

// 순서: Waist, Head_yaw, Head_pitch, L_sp,L_sr,L_sy,L_e, R_sp,R_sr,R_sy,R_e
// head 게인은 초기값(관성이 작아 팔보다 낮게 시작) - 실제 거동 보고 재튜닝 필요
double Kp[DoF] = { 1000, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 };
double Kd[DoF] = { 50,   30,  30,  1,   1,   1,   1,   1,   1,   1,   1   };

double PD_torque[DoF] = {0, };
double PD_acc[DoF] = {0, };

VectorXd gravity_torque = VectorXd::Zero(DoF);
VectorXd nonlinear_torque = VectorXd::Zero(DoF);
VectorXd dynamic_torque = VectorXd::Zero(DoF);

double target_torque[DoF] = {0, };

////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------- Impedance Control ---------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// 작업 단계: 접근 / 파지~내려놓기 / 복귀. 파지~내려놓기 구간에서만 임피던스 활성화.
// vision pick(command_mode==3) 실행 시 main.cpp가 세그먼트별로 dual_arm_phase_trajectory에
// 태깅해서 재생 중 자동으로 전환한다. 그 외 모드(0/1/2) 또는 idle 상태에서는
// /dual_arm/TaskPhase(std_msgs/Int32) 구독으로 수동 오버라이드 가능 (기본값: 접근, 임피던스 OFF).
enum TaskPhase { PHASE_APPROACH = 0, PHASE_GRASP_TO_PLACE = 1, PHASE_RETURN = 2 };
int task_phase = PHASE_APPROACH;

// F/T 센서 측정값 (force.x,y,z), /dual_arm/left_ft_sensor, /dual_arm/right_ft_sensor 콜백에서 갱신
Vector3d left_ft_force  = Vector3d::Zero();
Vector3d right_ft_force = Vector3d::Zero();

// 가상 스프링-댐퍼-질량 파라미터 (튜닝용): Md*e_ddot + Bd*e_dot + Kd*e = F_ext,  e = x_actual - x_desired
double Md_left[3]      = { 2.0, 2.0, 2.0 };      // 가상 질량 [kg]
double Bd_left[3]      = { 50.0, 50.0, 50.0 };   // 가상 댐핑 [N·s/m]
double Kd_imp_left[3]  = { 300.0, 300.0, 300.0 };// 가상 강성 [N/m]

double Md_right[3]     = { 2.0, 2.0, 2.0 };
double Bd_right[3]     = { 50.0, 50.0, 50.0 };
double Kd_imp_right[3] = { 300.0, 300.0, 300.0 };

const double IMPEDANCE_DLS_LAMBDA = 0.05;  // Cartesian 가속도 -> 관절 가속도 변환용 댐핑 의사역행렬 계수



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
                             MatrixXd& pos_out, MatrixXd& vel_out, MatrixXd& acc_out);

        // === 추가: 댐핑 의사역행렬 (mode 2에서 J -> J+ 변환용) ===
        MatrixXd DampedPinv(const MatrixXd& J, double lambda);     

};

DualArmControl dualarm; //class의 변수설정