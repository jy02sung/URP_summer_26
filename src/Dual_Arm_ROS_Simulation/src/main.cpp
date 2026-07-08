#include "dual_arm_function.cpp"
#include <ros/package.h> // 패키지 경로 자동 탐색용

// 알파벳 순서가 바뀌어도 안전하도록 이름으로 매칭하는 통합 콜백 함수
void msgCallbackJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "Waist_joint") {
            waist_jointp[0] = msg->position[i]; waist_jointv[0] = msg->velocity[i]; waist_torque[0] = msg->effort[i];
        }
        else if (msg->name[i] == "L_shoulder_pitch_joint") {
            left_arm_jointp[0] = msg->position[i]; left_arm_jointv[0] = msg->velocity[i]; left_arm_torque[0] = msg->effort[i];
        }
        else if (msg->name[i] == "L_shoulder_roll_joint") {
            left_arm_jointp[1] = msg->position[i]; left_arm_jointv[1] = msg->velocity[i]; left_arm_torque[1] = msg->effort[i];
        }
        else if (msg->name[i] == "L_shoulder_yaw_joint") {
            left_arm_jointp[2] = msg->position[i]; left_arm_jointv[2] = msg->velocity[i]; left_arm_torque[2] = msg->effort[i];
        }
        else if (msg->name[i] == "L_elbow_joint") {
            left_arm_jointp[3] = msg->position[i]; left_arm_jointv[3] = msg->velocity[i]; left_arm_torque[3] = msg->effort[i];
        }
        else if (msg->name[i] == "R_shoulder_pitch_joint") {
            right_arm_jointp[0] = msg->position[i]; right_arm_jointv[0] = msg->velocity[i]; right_arm_torque[0] = msg->effort[i];
        }
        else if (msg->name[i] == "R_shoulder_roll_joint") {
            right_arm_jointp[1] = msg->position[i]; right_arm_jointv[1] = msg->velocity[i]; right_arm_torque[1] = msg->effort[i];
        }
        else if (msg->name[i] == "R_shoulder_yaw_joint") {
            right_arm_jointp[2] = msg->position[i]; right_arm_jointv[2] = msg->velocity[i]; right_arm_torque[2] = msg->effort[i];
        }
        else if (msg->name[i] == "R_elbow_joint") {
            right_arm_jointp[3] = msg->position[i]; right_arm_jointv[3] = msg->velocity[i]; right_arm_torque[3] = msg->effort[i];
        }
        else if (msg->name[i] == "Head_yaw_joint") {
            head_jointp[0] = msg->position[i]; head_jointv[0] = msg->velocity[i]; head_torque[0] = msg->effort[i];
        }
        else if (msg->name[i] == "Head_pitch_joint") {
            head_jointp[1] = msg->position[i]; head_jointv[1] = msg->velocity[i]; head_torque[1] = msg->effort[i];
        }
    }
}

void msgCallbackDualArmCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // DoF(11) 만큼 받아오도록 수정
    for (int i = 0; i < DoF; i++) {
        dual_arm_commandp[i] = msg->data[i];
    }
    callback = true;
}

// Cartesian 좌표 수신용
double dual_arm_target_cart_L[3] = {0,};
double dual_arm_target_cart_R[3] = {0,};
bool cartesian_callback = false;

// Joint IK (곡선 기동) 좌표 수신용
double dual_arm_target_joint_ik_L[3] = {0,};
double dual_arm_target_joint_ik_R[3] = {0,};
bool joint_ik_callback = false;

bool is_cartesian_moving = false;

MatrixXd cartesian_p_trajectory_L, cartesian_v_trajectory_L, cartesian_a_trajectory_L;
MatrixXd cartesian_p_trajectory_R, cartesian_v_trajectory_R, cartesian_a_trajectory_R;


void msgCallbackCartesianCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 6) {
        dual_arm_target_cart_L[0] = msg->data[0]; dual_arm_target_cart_L[1] = msg->data[1]; dual_arm_target_cart_L[2] = msg->data[2];
        dual_arm_target_cart_R[0] = msg->data[3]; dual_arm_target_cart_R[1] = msg->data[4]; dual_arm_target_cart_R[2] = msg->data[5];
        cartesian_callback = true;
    }
}

// 단발성 IK (곡선 기동) 콜백
void msgCallbackJointIKCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 6) {
        dual_arm_target_joint_ik_L[0] = msg->data[0]; dual_arm_target_joint_ik_L[1] = msg->data[1]; dual_arm_target_joint_ik_L[2] = msg->data[2];
        dual_arm_target_joint_ik_R[0] = msg->data[3]; dual_arm_target_joint_ik_R[1] = msg->data[4]; dual_arm_target_joint_ik_R[2] = msg->data[5];
        joint_ik_callback = true;
    }
}

void msgCallbackPDGainCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() >= 2) {
        double new_P = msg->data[0];
        double new_D = msg->data[1];
        for (int i = 0; i < DoF; i++) {
            Kp[i] = new_P;
            Kd[i] = new_D;
        }
        ROS_INFO(">>> Real-time PD Gains Updated! Kp: %.2f, Kd: %.2f", new_P, new_D);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_control_main");
    ros::NodeHandle nh;
    
    #if ARMCTRLMODE == POSITION
        ros::Publisher dual_armjoint1_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint1_position_controller/command", 100);
        ros::Publisher dual_armjoint2_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint2_position_controller/command", 100);
        ros::Publisher dual_armjoint3_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint3_position_controller/command", 100);
        ros::Publisher dual_armjoint4_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint4_position_controller/command", 100);
        ros::Publisher dual_armjoint5_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint5_position_controller/command", 100);
        ros::Publisher dual_armjoint6_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint6_position_controller/command", 100);
        ros::Publisher dual_armjoint7_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint7_position_controller/command", 100);
        ros::Publisher dual_armjoint8_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint8_position_controller/command", 100);
        ros::Publisher dual_armjoint9_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint9_position_controller/command", 100);
        ros::Publisher dual_armjoint10_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint10_position_controller/command", 100); // 머리 Yaw
        ros::Publisher dual_armjoint11_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint11_position_controller/command", 100); // 머리 Pitch
    #elif ARMCTRLMODE == EFFORT
        ros::Publisher dual_armjoint1_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint1_effort_controller/command", 100);
        ros::Publisher dual_armjoint2_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint2_effort_controller/command", 100);
        ros::Publisher dual_armjoint3_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint3_effort_controller/command", 100);
        ros::Publisher dual_armjoint4_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint4_effort_controller/command", 100);
        ros::Publisher dual_armjoint5_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint5_effort_controller/command", 100);
        ros::Publisher dual_armjoint6_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint6_effort_controller/command", 100);
        ros::Publisher dual_armjoint7_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint7_effort_controller/command", 100);
        ros::Publisher dual_armjoint8_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint8_effort_controller/command", 100);
        ros::Publisher dual_armjoint9_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint9_effort_controller/command", 100);
        ros::Publisher dual_armjoint10_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint10_effort_controller/command", 100); // 머리 Yaw
        ros::Publisher dual_armjoint11_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint11_effort_controller/command", 100); // 머리 Pitch
    #endif

    // 하나로 통합된 조인트 스테이트 구독
    ros::Subscriber sub_joint_state = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackJointState);
    ros::Subscriber sub_dual_arm_cmd = nh.subscribe("/dual_arm/DualArmCmd_sim", 100, msgCallbackDualArmCmd);
    
    ros::Subscriber sub_cartesian_cmd = nh.subscribe("/dual_arm/CartesianCmd_sim", 100, msgCallbackCartesianCmd);
    ros::Subscriber sub_joint_ik_cmd = nh.subscribe("/dual_arm/JointIKCmd_sim", 100, msgCallbackJointIKCmd);
    ros::Subscriber sub_pd_gain_cmd = nh.subscribe("/dual_arm/PDGainCmd_sim", 10, msgCallbackPDGainCmd);
    
    ros::Rate loop_rate(1000);
    ros::spinOnce();
    
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg;
    std_msgs::Float64 head_yaw_joint_msg, head_pitch_joint_msg;

    // 절대경로 하드코딩 제거! 패키지 경로를 자동으로 찾습니다.
    string urdf_filename = ros::package::getPath("dual_arm") + "/urdf/dual_arm.urdf";
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");

    while(ros::ok())
    {
        dual_arm_jointp[0] = waist_jointp[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp[i + 1] = left_arm_jointp[i];
            dual_arm_jointp[i + 5] = right_arm_jointp[i];
        }
        dual_arm_jointp[9] = head_jointp[0];
        dual_arm_jointp[10] = head_jointp[1];

        dual_arm_jointv[0] = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv[i + 1] = left_arm_jointv[i];
            dual_arm_jointv[i + 5] = right_arm_jointv[i];
        }
        dual_arm_jointv[9] = head_jointv[0];
        dual_arm_jointv[10] = head_jointv[1];

        for (int i = 0; i < DoF; i++){
            dual_arm_jointv_lpf[i] = dualarm.LowPassFilter(dual_arm_jointv[i], dual_arm_jointv_before[i], 10);
            dual_arm_jointv_before[i] = dual_arm_jointv_lpf[i];
        }

        dual_arm_jointp_vec(0) = waist_jointp[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp_vec(i + 1) = left_arm_jointp[i];
            dual_arm_jointp_vec(i + 5) = right_arm_jointp[i];
        }
        dual_arm_jointp_vec(9) = head_jointp[0];
        dual_arm_jointp_vec(10) = head_jointp[1];

        dual_arm_jointv_vec(0) = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_vec(i + 1) = left_arm_jointv[i];
            dual_arm_jointv_vec(i + 5) = right_arm_jointv[i];
        }
        dual_arm_jointv_vec(9) = head_jointv[0];
        dual_arm_jointv_vec(10) = head_jointv[1];

        dual_arm_jointv_lpf_vec(0) = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_lpf_vec(i + 1) = left_arm_jointv[i];
            dual_arm_jointv_lpf_vec(i + 5) = right_arm_jointv[i];
        }
        dual_arm_jointv_lpf_vec(9) = head_jointv[0];
        dual_arm_jointv_lpf_vec(10) = head_jointv[1];

        // =========================================================================
        // 8번 메뉴 (관절 직접 입력 및 프리셋) 처리 블록
        // =========================================================================
        if(callback == true){
            is_cartesian_moving = false;

            for (int i = 0; i < DoF; i++) {
                dual_arm_initp[i] = dual_arm_jointp[i];
            }
            
            dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp, dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
            traj_cnt = 0;
            callback = false;
        }

        // =========================================================================
        // 9번 메뉴 (단발성 IK 후 Joint 곡선 기동) 처리 블록
        // =========================================================================
        else if (joint_ik_callback == true) {
            is_cartesian_moving = false;

            for (int i = 0; i < DoF; i++) {
                dual_arm_initp[i] = dual_arm_jointp[i];
            }

            cout << "\n[Joint IK Control] 단발성 양팔 IK 계산 시작 (Joint 곡선 기동)..." << endl;
            
            pinocchio::SE3 target_pose_L = pinocchio::SE3::Identity();
            target_pose_L.translation() << dual_arm_target_joint_ik_L[0], dual_arm_target_joint_ik_L[1], dual_arm_target_joint_ik_L[2];
            
            pinocchio::SE3 target_pose_R = pinocchio::SE3::Identity();
            target_pose_R.translation() << dual_arm_target_joint_ik_R[0], dual_arm_target_joint_ik_R[1], dual_arm_target_joint_ik_R[2];
            
            Eigen::VectorXd q_ik = dual_arm_jointp_vec;
            
            bool ik_success_L = dualarm.SolveIK_DLS(model, data, l_EE, target_pose_L, q_ik);
            bool ik_success_R = dualarm.SolveIK_DLS(model, data, r_EE, target_pose_R, q_ik);
            
            if (ik_success_L || ik_success_R) {
                cout << ">>> IK 연산 완료! Joint Space 곡선 궤적(Quintic)을 생성합니다." << endl;
                for (int i = 0; i < DoF; i++) {
                    dual_arm_commandp[i] = q_ik(i);
                }
                dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp, dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
                traj_cnt = 0;
            } else {
                cout << ">>> 양팔 모두 IK 수렴에 실패했습니다." << endl;
            }
            joint_ik_callback = false;
        }

        // =========================================================================
        // 0번 메뉴 (Cartesian 직선 궤적 계획)
        // =========================================================================
        else if (cartesian_callback == true) {
            is_cartesian_moving = true; 

            pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
            pinocchio::updateFramePlacements(model, data);
            
            Vector3d current_L = data.oMf[l_EE].translation();
            Vector3d current_R = data.oMf[r_EE].translation();

            Vector3d target_L(dual_arm_target_cart_L[0], dual_arm_target_cart_L[1], dual_arm_target_cart_L[2]);
            Vector3d target_R(dual_arm_target_cart_R[0], dual_arm_target_cart_R[1], dual_arm_target_cart_R[2]);
            
            cout << "\n[Cartesian Control] 양팔 직선 궤적(Linear Interpolation) 계산 시작..." << endl;
            
            dualarm.CartesianTrajectoryQuintic(current_L, target_L, cartesian_p_trajectory_L, cartesian_v_trajectory_L, cartesian_a_trajectory_L);
            dualarm.CartesianTrajectoryQuintic(current_R, target_R, cartesian_p_trajectory_R, cartesian_v_trajectory_R, cartesian_a_trajectory_R);
            
            traj_cnt = 0;
            cartesian_callback = false;
        }

        // =========================================================================
        // 0번: Cartesian 직선 궤적 추종 (매 틱 IK 계산)
        // =========================================================================
        else if (is_cartesian_moving && traj_cnt < cartesian_p_trajectory_L.rows()) {
            
            pinocchio::SE3 target_pose_L = pinocchio::SE3::Identity();
            target_pose_L.translation() << cartesian_p_trajectory_L(traj_cnt, 0), cartesian_p_trajectory_L(traj_cnt, 1), cartesian_p_trajectory_L(traj_cnt, 2);
            
            pinocchio::SE3 target_pose_R = pinocchio::SE3::Identity();
            target_pose_R.translation() << cartesian_p_trajectory_R(traj_cnt, 0), cartesian_p_trajectory_R(traj_cnt, 1), cartesian_p_trajectory_R(traj_cnt, 2);

            Eigen::VectorXd q_ik = dual_arm_jointp_vec;
            
            bool ik_success_L = dualarm.SolveIK_DLS(model, data, l_EE, target_pose_L, q_ik);
            bool ik_success_R = dualarm.SolveIK_DLS(model, data, r_EE, target_pose_R, q_ik);

            for (int i = 0; i < DoF; i++) {
                dual_arm_targetp[i] = q_ik(i);
                dual_arm_targetv[i] = 0.0; 
                dual_arm_targeta_vec(i) = 0.0;
            }

            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
            for (int i = 0; i < DoF; i++){
                dual_arm_targeta_vec(i) += PD_acc[i];
            }
            traj_cnt++;
        }

        // =========================================================================
        // 8번 & 9번: Joint Space 곡선 궤적 추종
        // =========================================================================
        else if (!is_cartesian_moving && traj_cnt < dual_arm_jointp_trajectory.rows()){
            for (int i = 0; i < DoF; i++){
                dual_arm_targetp[i] = dual_arm_jointp_trajectory(traj_cnt, i);
                dual_arm_targetv[i] = dual_arm_jointv_trajectory(traj_cnt, i);
            }
            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);

            for (int i = 0; i < DoF; i++){
                dual_arm_targeta_vec(i) = dual_arm_jointa_trajectory(traj_cnt, i) + PD_acc[i];
            }
            traj_cnt++;
        }

        // =========================================================================
        // 자세 유지(Hold) 로직
        // =========================================================================
        else {
            if (is_cartesian_moving) {
                // Cartesian 유지
            } else {
                for (int i = 0; i < DoF; i++) {
                    dual_arm_targetp[i] = dual_arm_jointp_trajectory.bottomRows(1)(0, i);
                }
            }
            for (int i = 0; i < DoF; i++) {
                dual_arm_targetv[i] = 0.0; 
                dual_arm_targeta_vec(i) = 0.0;
            }
            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
            for (int i = 0; i < DoF; i++) {
                dual_arm_targeta_vec(i) = PD_acc[i];
            }
        }

        for (int i = 0; i < DoF; ++i) {
            dual_arm_targetp_vec(i) = dual_arm_targetp[i];
        }

        gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_vec, dual_arm_targeta_vec);

        for (int i = 0; i < DoF; i++) {
            target_torque[i] = dynamic_torque(i);
        }
        
        pinocchio::forwardKinematics(model, data, dual_arm_targetp_vec);
        pinocchio::updateGlobalPlacements(model, data);
        pinocchio::updateFramePlacements(model, data);
        
        #if ARMCTRLMODE == POSITION
            waist_joint_msg.data            = dual_arm_targetp[0];
            shoulder_pitch_l_joint_msg.data = dual_arm_targetp[1];
            shoulder_roll_l_joint_msg.data  = dual_arm_targetp[2];
            shoulder_yaw_l_joint_msg.data   = dual_arm_targetp[3];
            elbow_l_joint_msg.data          = dual_arm_targetp[4];
            shoulder_pitch_r_joint_msg.data = dual_arm_targetp[5];
            shoulder_roll_r_joint_msg.data  = dual_arm_targetp[6];
            shoulder_yaw_r_joint_msg.data   = dual_arm_targetp[7];
            elbow_r_joint_msg.data          = dual_arm_targetp[8];
            head_yaw_joint_msg.data         = dual_arm_targetp[9];
            head_pitch_joint_msg.data       = dual_arm_targetp[10];
        #elif ARMCTRLMODE == EFFORT
            waist_joint_msg.data            = target_torque[0];
            shoulder_pitch_l_joint_msg.data = target_torque[1];
            shoulder_roll_l_joint_msg.data  = target_torque[2];
            shoulder_yaw_l_joint_msg.data   = target_torque[3];
            elbow_l_joint_msg.data          = target_torque[4];
            shoulder_pitch_r_joint_msg.data = target_torque[5];
            shoulder_roll_r_joint_msg.data  = target_torque[6];
            shoulder_yaw_r_joint_msg.data   = target_torque[7];
            elbow_r_joint_msg.data          = target_torque[8];
            head_yaw_joint_msg.data         = target_torque[9];
            head_pitch_joint_msg.data       = target_torque[10];
        #endif
    
        dual_armjoint1_pub.publish(waist_joint_msg);
        dual_armjoint2_pub.publish(shoulder_pitch_l_joint_msg);
        dual_armjoint3_pub.publish(shoulder_roll_l_joint_msg);
        dual_armjoint4_pub.publish(shoulder_yaw_l_joint_msg);
        dual_armjoint5_pub.publish(elbow_l_joint_msg);
        dual_armjoint6_pub.publish(shoulder_pitch_r_joint_msg);
        dual_armjoint7_pub.publish(shoulder_roll_r_joint_msg);
        dual_armjoint8_pub.publish(shoulder_yaw_r_joint_msg);
        dual_armjoint9_pub.publish(elbow_r_joint_msg);
        dual_armjoint10_pub.publish(head_yaw_joint_msg);
        dual_armjoint11_pub.publish(head_pitch_joint_msg);

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}