#include <iostream>
#include <fstream>
#include <csignal>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "ros/ros.h"
//--------------------MSG------------------------//
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace Eigen;
using Eigen::Quaternion;
#define PI 3.14159265358979323846
#define DEG2RAD PI/180
#define SAMPLING_TIME 0.001
#define SAMPLING_TIME_TRAJ 0.001
#define DoF 6

#define POSITION 1
#define TORQUE 2
#define ARMCTRLMODE TORQUE


#define EULER 1
#define QUAT 2
#define TRAJMODE QUAT

const double g = 9.81;

//--------------Flags-------------------//
bool first_callback = true;
bool init_flag = true;
bool t = false;
bool d = true;
bool traj = false;
bool ma = true;

//--------------Commands-------------------//
int arm_cmd_mode = 0;
// int mobile_cmd_mode = 0;
int step = 0;

double arm_jointp[DoF] = {0,};
double arm_jointv[DoF] = {0,};
double arm_commandp[DoF] = {0,};
double arm_initp[DoF] = {0, -PI, PI, 0, 0, 0};
double arm_initp_90[DoF] = {0, -PI, PI, 0, 0, PI/2};
double arm_targetp[DoF] = {0, };
double arm_target_torque[DoF] = {0, };
double arm_jointp_initial[DoF] = {0,};

double arm_jointv_before[DoF] = {0,};
double arm_jointv_lpf[DoF] = {0,};

double arm_jointp_before[DoF] = {0,};
double arm_jointp_lpf[DoF] = {0,};

//--------------Vision-------------------//
double pose[7] = {0,};
bool p = false;
bool a = false;
Vector3d eul_aruco = Vector3d::Zero(3);
VectorXd p_aruco = VectorXd::Zero(6);
VectorXd ab_aruco = VectorXd::Zero(6);
VectorXd mb_aruco = VectorXd::Zero(6);
VectorXd mb_aruco_before = VectorXd::Zero(6); 
VectorXd mb_aruco_lpf = VectorXd::Zero(6);
VectorXd T0cam = VectorXd::Zero(6);
int marker = 10;

//--------------Mobile-------------------//
double Mobile_p[3] = {0,};
double mobile_gazebop[3] = {0,};
double Mobile_v[3] = {0,};
double mobile_act[3] = {0,};
double mobile_commandp[3] = {0,};
double mobile_targetp[2] = {0,};
double mobile_targetp_1[2] = {0,};
MatrixXd mobile_out = MatrixXd::Zero(1,3);
MatrixXd mobile_out_1 = MatrixXd::Zero(1,3);
MatrixXd mobile_out_Z = MatrixXd::Zero(1,3);

//--------------Arm-------------------//
VectorXd arm_pose_act = VectorXd::Zero(6);
VectorXd arm_pose_quat_act = VectorXd::Zero(7);
VectorXd arm_pose_filtered = VectorXd::Zero(6);
VectorXd arm_pose_before = VectorXd::Zero(6);
VectorXd arm_pose_error = VectorXd::Zero(6);
VectorXd arm_traj_pose = VectorXd::Zero(6);
VectorXd arm_traj_pose_quat = VectorXd::Zero(7);
VectorXd arm_traj_pose_quat_before = VectorXd::Zero(7);
VectorXd arm_traj_pose_quat_vel = VectorXd::Zero(7);

VectorXd pose_ee = VectorXd::Zero(6);
VectorXd pose_tar = VectorXd::Zero(6);
VectorXd pose_tar_q = VectorXd::Zero(7);
Matrix4d T06_cmd = Matrix4d::Zero(4,4);
Matrix3d R06_cmd = Matrix3d::Zero(3,3);
Quaterniond q06_cmd(0,0,0,0);
Vector3d eul06_cmd = Vector3d::Zero(3);

//--------------IK-------------------//
// MatrixXd arm_targetp_multi = MatrixXd::Zero(2, 6);
MatrixXd J06 = MatrixXd::Zero(6,6);
MatrixXd J06_7 = MatrixXd::Zero(7,6);
MatrixXd inv_Jaco, inv_Jaco_1, I_damp;
Matrix4d T06 = Matrix4d::Zero(4,4);
Matrix3d R06 = Matrix3d::Zero(3,3);
Quaterniond q06(0,0,0,0);
Vector3d eul06 = Vector3d::Zero(3);
Matrix4d T06_act = Matrix4d::Identity(4,4);
Matrix4d T05_act = Matrix4d::Identity(4,4);
Matrix4d T5cam_act = Matrix4d::Identity(4,4);
Matrix4d T0cam_act = Matrix4d::Identity(4,4);
Matrix4d Tcamaruco_act = Matrix4d::Identity(4,4);
Matrix4d T0aruco_act = Matrix4d::Identity(4,4);
Matrix4d Tmb0_act = Matrix4d::Identity(4,4);
Matrix4d Tmbaruco_act = Matrix4d::Identity(4,4);
Matrix3d R06_act = Matrix3d::Zero(3,3);
Quaterniond q06_act(0,0,0,0);
Vector3d eul06_act = Vector3d::Zero(3);
Matrix4d Cur_HTM = Matrix4d::Zero(4,4);
VectorXd pose_error = VectorXd::Zero(3);
VectorXd angle_error = VectorXd::Zero(3);
double damp_const = 0;// 0.1;
double damp_const_pow = 0.001; // 0.001

//---------------Dynamic param---------------//
// warning index; i 0 = link 0, ... , i 6 = link 6 
MatrixXd omega_ = MatrixXd::Zero(3, 7);
MatrixXd d_omega_ = MatrixXd::Zero(3, 7);
MatrixXd a_ = MatrixXd::Zero(3, 7);
MatrixXd ac_ = MatrixXd::Zero(3, 7);
// i 7 = EE(dummy)
MatrixXd f_ = MatrixXd::Zero(3, 7);
MatrixXd n_ = MatrixXd::Zero(3, 7);
MatrixXd f_org = MatrixXd::Zero(3, 7);
MatrixXd n_org = MatrixXd::Zero(3, 7);
MatrixXd F_ = MatrixXd::Zero(3, 7);
MatrixXd N_ = MatrixXd::Zero(3, 7);
Vector3d vec3d_zeros = VectorXd::Zero(3);

//--------------Motion Planning-------------------//
bool mo = true;
bool tq = false;
MatrixXd pose_out_1 = MatrixXd::Zero(1,6);
MatrixXd pose_out_2 = MatrixXd::Zero(1,6);
MatrixXd pose_out_3 = MatrixXd::Zero(1,6);
int traj_cnt_1 = 2;
int traj_cnt_2 = 2;
int traj_cnt_3 = 2;
int traj_cnt_4 = 2;
int traj_cnt_5 = 2;
bool traj_1 = false;
bool traj_2 = false;
bool traj_3 = false;
bool traj_4 = false;
bool traj_5 = false;
VectorXd arm_traj_pose_1 = VectorXd::Zero(6);
VectorXd arm_traj_pose_2 = VectorXd::Zero(6);
VectorXd arm_traj_pose_3 = VectorXd::Zero(6);
VectorXd mani_cmd_1 = VectorXd::Zero(6);
VectorXd mani_cmd_2 = VectorXd::Zero(6);
VectorXd mani_cmd_3 = VectorXd::Zero(6);
double arm_jointp_initial_1[DoF] = {0,};
MatrixXd arm_jointp_trajectory_1 = MatrixXd::Zero(1,6);
MatrixXd arm_jointp_trajectory_2 = MatrixXd::Zero(1,6); 
MatrixXd arm_jointp_trajectory_3 = MatrixXd::Zero(1,6); 

//--------------Trajectory Planning-------------------//
bool traj_init = false;
bool b = false;
bool c = false;
int traj_cnt = 0;
int traj_size = 0;
int cnt = 2;
int cnt_1 = 2;
int cnt_z =2;
int cnt_a = 2;
int cnt_b = 2;
int cnt_c = 2;


MatrixXd arm_jointp_trajectory = MatrixXd::Zero(1,6); // resize later
MatrixXd arm_jointv_trajectory = MatrixXd::Zero(1,6);

VectorXd pose_ini = VectorXd::Zero(6);
VectorXd pose_cmd = VectorXd::Zero(6);
MatrixXd pose_out = MatrixXd::Zero(1,6);
VectorXd pose_quat_ini = VectorXd::Zero(7);
VectorXd pose_quat_cmd = VectorXd::Zero(7);
MatrixXd pose_quat_out = MatrixXd::Zero(1,7);


//--------------PID-------------------//
double arm_errorp[DoF] = {0, };
double PID_torque[DoF] = {0, };

// PD only
double  Kp[DoF] = {1000, 1000, 1000, 100, 200, 45},
		Ki[DoF] = {0, }, // 0.01
		Kd[DoF] = {5, 10, 10, 0.5, 1, 0.1};
// Model Base
// double  Kp[DoF] = {4000, 4000, 4000, 20000, 20000, 20000},
//         Ki[DoF] = {0, },
//         Kd[DoF] = {50, 50, 50, 100, 100, 100};

double  P_term[DoF] = {0, },
		I_term[DoF] = {0, },
        D_term[DoF] = {0, };

double gravity_torque[DoF] = {0, };
double dynamic_torque[DoF] = {0, };

class ArmControl
{
    private:
        //---------------Rotation---------------//
        Matrix<double, 4, 4> Ty90 { {0,  0, -1, 0},
                                    {0,  1,  0, 0},
                                    {1,  0,  0, 0},
                                    {0,  0,  0, 1} };

        Matrix<double, 4, 4> Tyn90 {{0,  0, 1, 0},
                                    {0,  1, 0, 0},
                                    {-1, 0, 0, 0},
                                    {0,  0, 0, 1} };

        //--------------DH param-------------------//
        const double d1 = 0.108;
        const double a2 = 0.35;
        const double a3 = 0.1;
        const double d4 = 0.4;
        const double d6 = 0.1;

        const double th_offset[DoF] = { 0, 	   0, -PI/2, 	 0,    0,    PI};
        const double d[DoF] = 		  {d1, 	   0, 	  0, 	d4,    0, 	 d6};
        const double a[DoF] = 		  { 0, 	   0,    a2, 	a3,    0, 	  0};
        const double alpha[DoF] = 	  { 0, -PI/2, 	  0, -PI/2, PI/2, -PI/2};

        const double th_cam[2] = { PI/2, -PI/2 };
        const double d_cam[2] = { -0.1, 0 };
        const double a_cam[2] = { 0.07, 0 };
        const double alpha_cam[2] = { PI/2, PI/2 };

        const double th_mb = 0;
        const double d_mb = 0.1;
        const double a_mb = 0.3;
        const double alpha_mb = 0;

        //--------------Inertial param-------------------//
        const double m0 = 1; // unit : kg
        const double m1 = 2;
        const double m2 = 2;
        const double m3 = 1.5;
        const double m4 = 1;
        const double m5 = 0.5;
        // const double m6 = 0.5; //5
        const double m6 = 0.95755;
        const double mass[7] = {m0, m1, m2, m3, m4, m5, m6};
        const double m_arm = m0 + m1 + m2 + m3 + m4 + m5 + m6;

        // CoM wrt local frame (DH parameter)
        const double x_b0 = 0.00000000     ,y_b0 = 0.00000000        ,z_b0 = 0.02237000; //unit: m ..
        const double x_b1 = 0.00000000     ,y_b1 = 0.00000000        ,z_b1 = -0.01083000; //unit: m ..
        const double x_b2 = 0.17500000     ,y_b2 = 0.00000000        ,z_b2 = 0.00000000; //unit: m ..
        const double x_b3 = 0.06978500     ,y_b3 = 0.06064000        ,z_b3 = 0.00000000; //unit: m ..
        const double x_b4 = 0.00000000     ,y_b4 = 0.00000000        ,z_b4 = -0.12241800; //unit: m ..
        const double x_b5 = 0.00000000     ,y_b5 = 0.03959500        ,z_b5 = 0.00000000; //unit: m ..
        // const double x_b6 = -0.00325200    ,y_b6 = 0.00000100        ,z_b6 = 0.07194400; //unit: m ..
        const double x_b6 = 0.000013       ,y_b6 = 0.000050          ,z_b6 = 0.078524; //unit: m ..
        Matrix<double, 7, 3> arm_CoM_pos_local {{x_b0,  y_b0,   z_b0},		// P0
                                                {x_b1,  y_b1,   z_b1},		// P1
                                                {x_b2,  y_b2,   z_b2},		// P2
                                                {x_b3,  y_b3,   z_b3},		// P3
                                                {x_b4,  y_b4,   z_b4},		// P4
                                                {x_b5,  y_b5,   z_b5},		// P5
                                                {x_b6,  y_b6,   z_b6} };	// P6

        const double I_xx[7] = {0.000705, 0.005014, 0.001909, 0.004752, 0.005564, 0.000958, 0.003287};
        const double I_yy[7] = {0.000705, 0.003034, 0.028177, 0.003243, 0.005636, 0.000481, 0.001655};
        const double I_zz[7] = {0.001025, 0.004383, 0.028631, 0.006701, 0.000697, 0.000889, 0.001979};
        const double I_xy[7] = {0.000000, 0.000000, 0.000000, -0.002269, 0.000000, 0.000000, 0.000000};
        Matrix<double, 3, 3> I0   { {I_xx[0],	0,          0		},
                                    {0,        	I_yy[0],   	0		},
                                    {0,        	0,          I_zz[0] } };
        Matrix<double, 3, 3> I1	  { {I_xx[1],	0,          0		},
                                    {0,        	I_yy[1],   	0		},
                                    {0,        	0,          I_zz[1] } };
        Matrix<double, 3, 3> I2   { {I_xx[2],	0,          0		},
                                    {0,        	I_yy[2],   	0		},
                                    {0,        	0,          I_zz[2] } };
        Matrix<double, 3, 3> I3	  { {I_xx[3],	I_xy[3],    0		},
                                    {I_xy[3],   I_yy[3],   	0		},
                                    {0,        	0,          I_zz[3] } };
        Matrix<double, 3, 3> I4	  { {I_xx[4],	0,          0		},
                                    {0,        	I_yy[4],   	0		},
                                    {0,        	0,          I_zz[4] } };
        Matrix<double, 3, 3> I5	  { {I_xx[5],	0,          0		},
                                    {0,        	I_yy[5],   	0		},
                                    {0,        	0,          I_zz[5] } };
        Matrix<double, 3, 3> I6   { {I_xx[6],	0,          0		},
                                    {0,        	I_yy[6],   	0		},
                                    {0,        	0,          I_zz[6] } };

    public:
        ArmControl();
        ~ArmControl();

        // Utility
        int sgn(double x);
        Matrix3d scew(Ref<Matrix<double, 3, 1>> vec_);
        double LowPassFilter(double input, double output_before, double cutoff_frequency);
        void Ry180(Matrix3d& R_);

        // Rotation
        void Qua2Euler(Quaterniond& q, Vector3d& eul);
        void Euler2Qua(Vector3d& eul, Quaterniond& q);
        void Rot2Qua(Matrix3d& R, Quaterniond& q);
        void Eul2Rot(Vector3d& eul, Matrix3d& R);

        // Transformation
        Matrix4d T_craig(float th, float d, float al, float a);
        void Joint2Cartesian(double* th, VectorXd& pose);
        void Joint2CartesianQuat(double* th, VectorXd& pose_quat);
        void PoseEul2Tf(const VectorXd& pose, Ref<Matrix<double, 4, 4>> T_);
        void Tf2PoseEul(const Matrix<double, 4, 4>& T_, VectorXd& pose);
        void PoseEuler2Quat(VectorXd& pose_eul, VectorXd& pose_quat);
        void PoseQuat2Euler(VectorXd& pose_quat, VectorXd& pose_eul);
        void ab_T_link5(double* currentpos, Matrix4d& T);
        void link5_T_cam(Matrix4d& T);
        void mb_T_ab(Matrix4d& T);

        // Kinematics
        void forwardKinematics(double* currentpos, Matrix4d& T);
        void inverseKinematics(VectorXd& target_pose, double* currentpos, double* th_out);
        void Inverse_Newton(VectorXd& target_pose_, double* currentpos_, VectorXd& pose_act_, double* th_out_);
        void Inverse_Newton_Quat(VectorXd& target_pose_, double* currentpos_, double* th_out_);

        // Jacobian
        void Jacobian(double* th, MatrixXd& J06);
        void Angvel2Eul(Vector3d& eul, MatrixXd& J06);
        void Angvel2Quat(Quaterniond& q, MatrixXd& J06, MatrixXd& J06_7);

        // Dynamics
        void RNEA(const double* q, const double* dq, const double* ddq, Ref<Matrix<double, 3, 1>> omega_b, Ref<Matrix<double, 3, 1>> acc_b, Ref<Matrix<double, 3, 1>> grav_local, double* arm_torque);
        void MassMatrix(const double* th, MatrixXd& mass_matrix);
        void GravityTorque(double* currentpos, double* gravity_torque);

        // Trajectory
        void JointTrajectory(double* th_ini, double* th_cmd, MatrixXd& th_out);
        void JointTrajectoryTrapezoidal(double* th_ini, double* th_cmd, MatrixXd& th_out, MatrixXd& vel_out);
        void CartesianTrajectoryEuler(VectorXd& pose_ini, VectorXd& pose_cmd, MatrixXd& pose_out); // , MatrixXd& vel_out);
        void CartesianTrajectoryQuat(VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, MatrixXd& pose_quat_out);
        void CartesianTrajectoryTrapezoidal(VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, MatrixXd& pose_quat_out);

        // Controller
        void PIDController(double* targetpos_, double* currentpos, double* currentvel_, double* PDtorque_);
     
        //Lerp
        void Lerp(VectorXd& S_out, VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, int step,  MatrixXd& pose_quat_out);

        //Slerp
        // void Slerp(VectorXd& S_out, VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, int step, double quat_angle, MatrixXd& pose_quat_out);

        void MobileTrajectory(double* Mobile_ini, double* Mobile_cmd, MatrixXd& Mobile_out);
        void MobileTrajectory_Z(double* Mobile_ini, double* Mobile_cmd, MatrixXd& Mobile_out);
};

ArmControl arm;