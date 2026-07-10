#include "dual_arm_function.cpp"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        waist_jointp[0] = msg->position[8];
        waist_jointv[0] = msg->velocity[8]; 
        waist_torque[0] = msg->effort[8];
    }

void msgCallbackLeftArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        left_arm_jointp[0] = msg->position[1];
        left_arm_jointv[0] = msg->velocity[1];
        left_arm_torque[0] = msg->effort[1];

        left_arm_jointp[1] = msg->position[2];
        left_arm_jointv[1] = msg->velocity[2];
        left_arm_torque[1] = msg->effort[2];

        left_arm_jointp[2] = msg->position[3];
        left_arm_jointv[2] = msg->velocity[3];
        left_arm_torque[2] = msg->effort[3];

        left_arm_jointp[3] = msg->position[0];
        left_arm_jointv[3] = msg->velocity[0];
        left_arm_torque[3] = msg->effort[0];
    }

void msgCallbackRightArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        right_arm_jointp[0] = msg->position[5];
        right_arm_jointv[0] = msg->velocity[5];
        right_arm_torque[0] = msg->effort[5];
    
        right_arm_jointp[1] = msg->position[6];
        right_arm_jointv[1] = msg->velocity[6];
        right_arm_torque[1] = msg->effort[6];
        
        right_arm_jointp[2] = msg->position[7];
        right_arm_jointv[2] = msg->velocity[7];
        right_arm_torque[2] = msg->effort[7];
        
        right_arm_jointp[3] = msg->position[4];
        right_arm_jointv[3] = msg->velocity[4];
        right_arm_torque[3] = msg->effort[4];
    }

void msgCallbackDualArmCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for (int i = 0; i < 9; i++) {
            dual_arm_commandp[i] = msg-> data[i];
        }
        callback = true;
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main 함수
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_control_main");
    ros::NodeHandle nh;
    
    // ROS 퍼블리셔 설정 (각 관절에 해당하는 토픽)
    //Position Cotrol
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

    //Effort Control
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
    
    #endif

    // ROS 서브스크라이버 설정 (조인트 상태 토픽)
    ros::Subscriber sub_waist_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackWaistArmJointState);
    ros::Subscriber sub_left_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackLeftArmJointState);
    ros::Subscriber sub_right_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackRightArmJointState);
    ros::Subscriber sub_dual_arm_cmd = nh.subscribe("/dual_arm/DualArmCmd_sim", 100, msgCallbackDualArmCmd);
   
    ros::Rate loop_rate(1000);
    ros::spinOnce();
    
    // 조인트 명령 메시지 객체 선언
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg;

    // 고정된 URDF 파일 경로 사용
    string urdf_filename = "/home/seungjun/catkin_ws/src/dual_arm/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(ros::ok())
    {

        dual_arm_jointp[0] = waist_jointp[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp[i + 1] = left_arm_jointp[i];
            dual_arm_jointp[i + 5] = right_arm_jointp[i];
        }

        dual_arm_jointv[0] = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv[i + 1] = left_arm_jointv[i];
            dual_arm_jointv[i + 5] = right_arm_jointv[i];
        }

        for (int i = 0; i < DoF; i++){
            dual_arm_jointv_lpf[i] = dualarm.LowPassFilter(dual_arm_jointv[i], dual_arm_jointv_before[i], 10);
            dual_arm_jointv_before[i] = dual_arm_jointv_lpf[i];
        }

        // dual_arm_initp[0] = waist_jointp[0];
        // for (int i = 0; i < 4; i++) {
        //     dual_arm_initp[i + 1] = left_arm_jointp[i];
        //     dual_arm_initp[i + 5] = right_arm_jointp[i];
        // }

        dual_arm_jointp_vec(0) = waist_jointp[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp_vec(i + 1) = left_arm_jointp[i];
            dual_arm_jointp_vec(i + 5) = right_arm_jointp[i];
        }

        dual_arm_jointv_vec(0) = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_vec(i + 1) = left_arm_jointv[i];
            dual_arm_jointv_vec(i + 5) = right_arm_jointv[i];
        }

        dual_arm_jointv_lpf_vec(0) = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_lpf_vec(i + 1) = left_arm_jointv[i];
            dual_arm_jointv_lpf_vec(i + 5) = right_arm_jointv[i];
        }



        if(callback == true){
            dual_arm_initp[0] = waist_jointp[0];
            for (int i = 0; i < 4; i++) {
                dual_arm_initp[i + 1] = left_arm_jointp[i];
                dual_arm_initp[i + 5] = right_arm_jointp[i];
            }
            // dualarm.JointTrajectoryTrapezoidal(dual_arm_initp, dual_arm_commandp, dual_arm_jointp_trajectory, dual_arm_jointv_trajectory);
            dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp, dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
            traj_cnt = 0;
            callback = false;
        }
        else if (traj_cnt < dual_arm_jointp_trajectory.rows()){
            for (int i = 0; i < DoF; i++){
                dual_arm_targetp[i] = dual_arm_jointp_trajectory(traj_cnt, i);
            }
            for (int i = 0; i < DoF; i++){
                dual_arm_targetv[i] = dual_arm_jointv_trajectory(traj_cnt, i);
            }
            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);

            for (int i = 0; i < DoF; i++){
                dual_arm_targeta_vec(i) = dual_arm_jointa_trajectory(traj_cnt, i) + PD_acc[i];
            }
            traj_cnt++;
        }
        else {
            // 마지막 타겟 자세 유지
            for (int i = 0; i < DoF; i++) {
                dual_arm_targetp[i] = dual_arm_jointp_trajectory.bottomRows(1)(0, i);
                dual_arm_targetv[i] = 0.0;  // 정지 목표
                dual_arm_targeta_vec(i) = 0.0;
            }
        
            // PD 제어로 자세 유지
            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
        
            for (int i = 0; i < DoF; i++) {
                dual_arm_targeta_vec(i) = PD_acc[i];
            }
        
        }


        for (int i = 0; i < DoF; ++i) {
            dual_arm_targetp_vec(i) = dual_arm_targetp[i];
        }

        // dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_jointv, PD_torque);
        gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_vec, dual_arm_targeta_vec);

        // dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_jointv_lpf, PD_torque);
        // gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        // dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_lpf_vec, dual_arm_targeta_vec);

        for (int i = 0; i < DoF; i++) {
            target_torque[i] = dynamic_torque(i);
        }

        // for(int i = 0; i < DoF; i++){
        //     target_torque[i] = PD_torque[i];
        // }
        
        // Forward Kinematics 수행(Vector만 입력가능)
        pinocchio::forwardKinematics(model, data, dual_arm_targetp_vec);
        // 모델의 전역 조인트 위치/자세 업데이트 (oMi[i] 사용가능)
        pinocchio::updateGlobalPlacements(model, data);
        // 프레임(예: 엔드 이펙터)의 위치와 자세 업데이트 (oMf[i] 사용가능)
        pinocchio::updateFramePlacements(model, data);
        // Joint의 위치와 자세
        pinocchio::JointIndex waist =  model.getJointId("Waist_joint");
        pinocchio::JointIndex l_sp  =  model.getJointId("L_shoulder_pitch_joint");
        pinocchio::JointIndex l_sr  =  model.getJointId("L_shoulder_roll_joint");
        pinocchio::JointIndex l_sy  =  model.getJointId("L_shoulder_yaw_joint");
        pinocchio::JointIndex l_e   =  model.getJointId("L_elbow_joint");
        pinocchio::JointIndex r_sp  =  model.getJointId("R_shoulder_pitch_joint");
        pinocchio::JointIndex r_sr  =  model.getJointId("R_shoulder_roll_joint");
        pinocchio::JointIndex r_sy  =  model.getJointId("R_shoulder_yaw_joint");
        pinocchio::JointIndex r_e   =  model.getJointId("R_elbow_joint");
        // EE의 위치와 자세
        pinocchio::FrameIndex l_EE  =  model.getFrameId("L_EE_joint");
        pinocchio::FrameIndex r_EE  =  model.getFrameId("R_EE_joint");

        // Position Control
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

        // Effort Control
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

        #endif
    
        // ROS 토픽으로 조인트 명령 발행
        dual_armjoint1_pub.publish(waist_joint_msg);
        dual_armjoint2_pub.publish(shoulder_pitch_l_joint_msg);
        dual_armjoint3_pub.publish(shoulder_roll_l_joint_msg);
        dual_armjoint4_pub.publish(shoulder_yaw_l_joint_msg);
        dual_armjoint5_pub.publish(elbow_l_joint_msg);
        dual_armjoint6_pub.publish(shoulder_pitch_r_joint_msg);
        dual_armjoint7_pub.publish(shoulder_roll_r_joint_msg);
        dual_armjoint8_pub.publish(shoulder_yaw_r_joint_msg);
        dual_armjoint9_pub.publish(elbow_r_joint_msg);

        
        cout << fixed << setprecision(4);

        // cout << dual_arm_jointp[0]*rad2deg << endl;
        // cout << dual_arm_jointp[1]*rad2deg << "   " << dual_arm_jointp[5]*rad2deg << endl;
        // cout << dual_arm_jointp[2]*rad2deg << "   " << dual_arm_jointp[6]*rad2deg << endl;
        // cout << dual_arm_jointp[3]*rad2deg << "   " << dual_arm_jointp[7]*rad2deg << endl;
        // cout << dual_arm_jointp[4]*rad2deg << "   " << dual_arm_jointp[8]*rad2deg << endl;
        

        cout << "=== Left End-Effector Pose ===" << endl;
        cout << "Position: " << data.oMf[l_EE].translation().transpose() << endl;
        cout << "Orientation (Rotation Matrix):\n" << data.oMf[l_EE].rotation() << endl;
        
        cout << "=== Right End-Effector Pose ===" << endl;
        cout << "Position: " << data.oMf[r_EE].translation().transpose() << endl;
        cout << "Orientation (Rotation Matrix):\n" << data.oMf[r_EE].rotation() << endl;     

        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}