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
#include <deque>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/WrenchStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;
using namespace Eigen;

#define POSITION 1
#define EFFORT 2
#define ARMCTRLMODE 2


// 전역 변수 선언
const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;
const int DoF = 11;
const double SAMPLING_TIME = 0.001;
const double SAMPLING_TIME_TRAJ = 0.001;

geometry_msgs::PoseStamped aruco_marker_pose;
bool aruco_pose_received = false;
Vector3d marker_pos_world = Vector3d::Zero();

bool use_hardcoded_box_position = true;

int command_mode = 0;   // 0:modeling(관절각) 1:joint sim(IK+5차) 2:cartesian sim(직선)
double dual_arm_commandx[6] = {0,};   // 목표 EE 위치 [Lx,Ly,Lz,Rx,Ry,Rz]
MatrixXd dual_arm_cart_pos_trajectory = MatrixXd::Zero(1,6);  // 직선 궤적(직교)
MatrixXd dual_arm_cart_vel_trajectory = MatrixXd::Zero(1,6);  // 직교 속도
MatrixXd dual_arm_cart_acc_trajectory = MatrixXd::Zero(1,6);  // 직교 가속도
VectorXd q_ik_seed = VectorXd::Zero(DoF);   // IK 시드
VectorXd q_ik_result = VectorXd::Zero(DoF); // IK 결과

// 우리 배열 인덱스(0~10) → Pinocchio q 인덱스로 변환하는 테이블
// ※ 코드 내부 순서는 "우리 코드 배열 순서"입니다. Pinocchio 내부 q 순서와 다르므로
//    Pinocchio 함수(IK, FK, RNEA) 호출 시에는 반드시 convertToPinocchioOrder/
//    convertFromPinocchioOrder로 변환해서 사용해야 합니다.
// ourIdx_to_pinocchioIdx = {0, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2}
//   우리[0]=Waist        → Pinocchio q(0)
//   우리[1]=L_sp         → Pinocchio q(3)
//   우리[2]=L_sr         → Pinocchio q(4)
//   우리[3]=L_sy         → Pinocchio q(5)
//   우리[4]=L_e          → Pinocchio q(6)
//   우리[5]=R_sp         → Pinocchio q(7)
//   우리[6]=R_sr         → Pinocchio q(8)
//   우리[7]=R_sy         → Pinocchio q(9)
//   우리[8]=R_e          → Pinocchio q(10)
//   우리[9]=Head_yaw     → Pinocchio q(1)
//   우리[10]=Head_pitch  → Pinocchio q(2)
const int ourIdx_to_pinocchioIdx[DoF] = {0, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2};

bool callback = false;
int traj_cnt = 1;

enum GraspState {
    GRASP_IDLE,
    GRASP_APPROACH_JOINT,
    GRASP_APPROACH_LINE,
    GRASP_CONTACT_WAIT,
    GRASP_HOLDING,
    GRASP_LIFT,
    GRASP_ROTATE,
    GRASP_PLACE,
    GRASP_RELEASE
};

GraspState grasp_state = GRASP_IDLE;

// 파지 목표점 저장용 (2단계에서도 재사용)
Vector3d grasp_goalL_approach = Vector3d::Zero();
Vector3d grasp_goalR_approach = Vector3d::Zero();
Vector3d grasp_goalL_touch    = Vector3d::Zero();
Vector3d grasp_goalR_touch    = Vector3d::Zero();

// 각 관절 상태 저장 (필요 시 콜백 함수에서 사용)
double waist_jointp[1] = {0};
double waist_jointv[1] = {0};
double waist_torque[1] = {0};
double left_arm_jointp[4] = {0,};
double left_arm_jointv[4] = {0,};
double left_arm_torque[4] = {0,};
double right_arm_jointp[4] = {0,};
double right_arm_jointv[4] = {0,};
double right_arm_torque[4] = {0,};

double head_jointp[2] = {0,};
double head_jointv[2] = {0,};
double head_torque[2] = {0,};

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

// 인덱스: 0=Waist, 1=L_shoulder_pitch, 2=L_shoulder_roll, 3=L_shoulder_yaw, 4=L_elbow,
//         5=R_shoulder_pitch, 6=R_shoulder_roll, 7=R_shoulder_yaw, 8=R_elbow,
//         9=Head_yaw, 10=Head_pitch
double Kp[DoF] = { 1000, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 };
//                 Waist  Lsp  Lsr  Lsy   Le  Rsp  Rsr  Rsy   Re   Hy   Hp

double Kd[DoF] = {   50,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1 };
//                 Waist  Lsp  Lsr  Lsy   Le  Rsp  Rsr  Rsy   Re   Hy   Hp

double PD_torque[DoF] = {0, };
double PD_acc[DoF] = {0, };

VectorXd gravity_torque = VectorXd::Zero(DoF);
VectorXd nonlinear_torque = VectorXd::Zero(DoF);
VectorXd dynamic_torque = VectorXd::Zero(DoF);

double target_torque[DoF] = {0, };

Vector3d l_ft_force = Vector3d::Zero();
Vector3d r_ft_force = Vector3d::Zero();
Vector3d l_ft_torque = Vector3d::Zero();
Vector3d r_ft_torque = Vector3d::Zero();

Vector3d grasp_current_goalL = Vector3d::Zero();   // 힘 제어 중 실시간으로 조정되는 목표
Vector3d grasp_current_goalR = Vector3d::Zero();
double force_control_gain = 0.0005;   // Kf, [m/N] 튜닝 필요
double target_grasp_force = 30.0;         // [N] 목표 파지력
double contact_force_threshold = 1.0;     // [N] 접촉 감지 임계값

double l_contact_force_filtered = 0.0;
double r_contact_force_filtered = 0.0;

double l_force_fast = 0.0;
double r_force_fast = 0.0;

double l_contact_force = 0.0;
double r_contact_force = 0.0;

bool grasp_reached_target = false;   // 양팔 공통

std::deque<double> l_force_history;
std::deque<double> r_force_history;
const int MEDIAN_WINDOW = 7;

double grasp_locked_forceL = 0.0;
double grasp_locked_forceR = 0.0;

// ── y축 어드미턴스 제어 (K=0, 힘 목표 추종) ──
double M_d = 1;
double D_d = 50.0;
double K_d_adm = 900.0;   // 어드미턴스 y축 가상 강성 (물체 kp=30000보다 커야 함)
double y_L = 0.0, y_dot_L = 0.0;
double y_R = 0.0, y_dot_R = 0.0;

double integral_L = 0.0;
double integral_R = 0.0;
double K_i = 0.0;   // 적분 게인, 튜닝 필요

bool integral_active_L = false;
bool integral_active_R = false;

// ── x, z축 안정화 (K 포함, 원위치 복원) ──
double M_xz = 1.0;
double D_xz = 260 ;
double K_xz = 400.0;
double x_L = 0.0, x_dot_L = 0.0;
double z_L = 0.0, z_dot_L = 0.0;
double x_R = 0.0, x_dot_R = 0.0;
double z_R = 0.0, z_dot_R = 0.0;

double lift_height = 0.1;        //  들어올리기
double grasp_lift_startZ = 0.0;   // 들어올리기 시작 z 높이

double place_height = 0.1;        // 내려놓기 (lift_height와 별개 변수, 지금은 같은 값)
double box_center_z_place_start = 0.0;   // PLACE 시작 시점 z 높이 (여기서 place_height만큼 내려감)

double release_offset = 0.0;      // RELEASE 중 양쪽으로 벌어지는 거리 (0 → release_distance)
double release_distance = 0.05;   // 파지 해제 시 바깥쪽으로 벌리는 거리 [m]
double release_speed_start = 0.005;   // 시작 속도 [m/s] — 천천히 빠지면서 마찰로 자연스럽게 내려앉게
double release_speed_end = 0.05;      // 최종 속도 [m/s] — 어느정도 빠진 뒤엔 빠르게 마무리

double box_mass = 1.0;   // 박스 질량 [kg]
double gravity = 9.81;

// SE3 T_world_A = data.oMf[A];   // 또는 oMi[A]도 가능
// SE3 T_world_B = data.oMf[B];

// SE3 T_A_B = T_world_A.inverse() * T_world_B; 기준프레임을 world에서 A로 변환하기 (A프레임 기준으로 본B의 pose)

VectorXd q_bias_pin = VectorXd::Zero(DoF);   // 좋은 시작 자세 (Pinocchio 순서)
double k_null = 0.0;   // null-space 게인, 튜닝 필요

//double EE_weight = 0.15 * 9.81;   // L_EE, R_EE 링크 자체 무게 (약 1.4715N)
double EE_weight = 1.47;
double l_force_x_fast = 0.0;
double r_force_x_fast = 0.0;

double l_force_z_fast = 0.0;
double r_force_z_fast = 0.0;

double l_force_z_locked = 0.0;
double r_force_z_locked = 0.0;
bool z_force_locked = false;

double waist_rotate_target = 0.0;   // 목표 허리 각도 (라디안)

double waist_angle_locked = 0.0;

// z축(허리) 기준 회전 변환 함수
inline Vector3d RotateZ(const Vector3d& v, double theta)
{
    double c = cos(theta), s = sin(theta);
    return Vector3d(c*v(0) - s*v(1), s*v(0) + c*v(1), v(2));
}

inline Vector3d InverseRotateZ(const Vector3d& v, double theta)
{
    return RotateZ(v, -theta);   // 역회전은 -theta
}

Vector3d box_center_world = Vector3d::Zero();   // 마커로 인식한 박스 중심 (월드 기준, 고정값)
double waist_ref_angle = 0.0;                    // GRASP_HOLDING 시작 시점의 허리각 (기준점)

double box_width = 0.3;
double EE_radius = 0.0325;

double box_center_z_start = 0.0;

double grasp_z_offset = -0.05;

double waist_target_prev = 0.0;
bool waist_target_initialized = false;

double waist_theta_goal = 0.0;   // 회전 목표각 (몸 기준, 서서히 증가)

Vector3d waist_axis_world = Vector3d(0, 0, 0.943);   // 허리 회전축 월드 위치 (xy만 중요)
Vector3d box_offset_from_axis = Vector3d::Zero();     // 그랩 시작 시점의 "축→박스" 벡터

Vector3d ee_L_offset_from_axis = Vector3d::Zero();
Vector3d ee_R_offset_from_axis = Vector3d::Zero();

double waist_ref_angle_at_rotate_start = 0.0;

double theta_now_common = 0.0;   // 목표(계획)용 각도 — 목표 위치 계산 전용
double theta_now_actual = 0.0;   // 실측 각도 — 힘 센서 변환/반력 보상 전용

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
                      const VectorXd& q_seed, VectorXd& q_out);   // 기존 버전 (그대로 유지)

        void SolveIK_Position(pinocchio::Model& model, pinocchio::Data& data,
                            pinocchio::FrameIndex l_EE, pinocchio::FrameIndex r_EE,
                            const Vector3d& target_L, const Vector3d& target_R,
                            const VectorXd& q_seed, VectorXd& q_out,
                            const VectorXd& q_bias, double k_null,
                            bool freeze_waist = false, double waist_lock_value = 0.0);    // 새 버전 (null-space 포함)
        

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