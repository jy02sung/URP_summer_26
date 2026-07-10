#ifndef DUAL_ARM_FUNCTION_H
#define DUAL_ARM_FUNCTION_H

#include "pinocchio/parsers/urdf.hpp"          
#include "pinocchio/algorithm/joint-configuration.hpp" 
#include "pinocchio/algorithm/kinematics.hpp"   
#include "pinocchio/algorithm/frames.hpp"       
#include "pinocchio/algorithm/jacobian.hpp"     
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <algorithm>  
#include <vector>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace Eigen;

#define POSITION 1
#define EFFORT 2
#define ARMCTRLMODE 1

// 전역 변수 선언
const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;

// 머리의 Yaw, Pitch 관절 2개가 추가되어 자유도를 11로 변경합니다.
const int DoF = 11; 
const double SAMPLING_TIME = 0.001;
const double SAMPLING_TIME_TRAJ = 0.001;

bool callback = false;
int traj_cnt = 1;

// 각 관절 상태 저장
double waist_jointp[1] = {0};
double waist_jointv[1] = {0};
double waist_torque[1] = {0};
double left_arm_jointp[4] = {0,};
double left_arm_jointv[4] = {0,};
double left_arm_torque[4] = {0,};
double right_arm_jointp[4] = {0,};
double right_arm_jointv[4] = {0,};
double right_arm_torque[4] = {0,};

// 머리 관절용 상태 변수 추가
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

// RNEA torque control uses the same conservative gains previously used by
// Gazebo's position controllers.  The old 500/1 values are unstable when
// applied directly as desired joint accelerations.
double Kp[DoF] = { 45, 45, 45, 45, 45, 45, 45, 45, 45, 20, 20 };
double Kd[DoF] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 2.5, 2.5 };

double PD_torque[DoF] = {0, };
double PD_acc[DoF] = {0, };

class DualArmControl
{
    public:
        DualArmControl();
        ~DualArmControl();

        double LowPassFilter(double &input, double &output_before, double cutoff_frequency);
        void JointTrajectoryTrapezoidal(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out);
        void JointTrajectoryQuintic(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out, MatrixXd& q_acc_out);
        
        // Cartesian 직선 궤적 생성기 (XYZ 3차원)
        void CartesianTrajectoryQuintic(const Vector3d& p_ini, const Vector3d& p_cmd, MatrixXd& p_out, MatrixXd& p_dot_out, MatrixXd& p_acc_out);

        void PDController(double* target_q, double* current_q, double* target_q_dot, double* current_q_dot, double* PDtorque);
        bool SolveIK_DLS(pinocchio::Model& model, pinocchio::Data& data, const pinocchio::FrameIndex frame_id, const pinocchio::SE3& target_pose, Eigen::VectorXd& q_inout);
        bool SolvePositionIK_DLS(pinocchio::Model& model, pinocchio::Data& data, const pinocchio::FrameIndex frame_id, const Eigen::Vector3d& target_position, const std::vector<int>& active_indices, Eigen::VectorXd& q_inout);
        void ApplyArmPostureConstraint(Eigen::VectorXd& q_inout, bool is_left_arm) const;
};

DualArmControl dualarm; 

#endif
