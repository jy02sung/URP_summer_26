#include "dual_arm_function.cpp"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
void msgCallbackHeadJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    head_jointp[0] = msg->position[1];   // yaw는 인덱스 1
    head_jointv[0] = msg->velocity[1];
    head_torque[0] = msg->effort[1];

    head_jointp[1] = msg->position[0];   // pitch는 인덱스 0
    head_jointv[1] = msg->velocity[0];
    head_torque[1] = msg->effort[0];
}

void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    waist_jointp[0] = msg->position[10];   // Waist_joint
    waist_jointv[0] = msg->velocity[10]; 
    waist_torque[0] = msg->effort[10];
}

void msgCallbackLeftArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    left_arm_jointp[0] = msg->position[3];   // L_shoulder_pitch_joint
    left_arm_jointv[0] = msg->velocity[3];
    left_arm_torque[0] = msg->effort[3];

    left_arm_jointp[1] = msg->position[4];   // L_shoulder_roll_joint
    left_arm_jointv[1] = msg->velocity[4];
    left_arm_torque[1] = msg->effort[4];

    left_arm_jointp[2] = msg->position[5];   // L_shoulder_yaw_joint
    left_arm_jointv[2] = msg->velocity[5];
    left_arm_torque[2] = msg->effort[5];

    left_arm_jointp[3] = msg->position[2];   // L_elbow_joint
    left_arm_jointv[3] = msg->velocity[2];
    left_arm_torque[3] = msg->effort[2];
}

void msgCallbackRightArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
    right_arm_jointp[0] = msg->position[7];   // R_shoulder_pitch_joint
    right_arm_jointv[0] = msg->velocity[7];
    right_arm_torque[0] = msg->effort[7];

    right_arm_jointp[1] = msg->position[8];   // R_shoulder_roll_joint
    right_arm_jointv[1] = msg->velocity[8];
    right_arm_torque[1] = msg->effort[8];

    right_arm_jointp[2] = msg->position[9];   // R_shoulder_yaw_joint
    right_arm_jointv[2] = msg->velocity[9];
    right_arm_torque[2] = msg->effort[9];

    right_arm_jointp[3] = msg->position[6];   // R_elbow_joint
    right_arm_jointv[3] = msg->velocity[6];
    right_arm_torque[3] = msg->effort[6];
}
void msgCallbackLeftFT(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    l_ft_force(0) = msg->wrench.force.x;
    l_ft_force(1) = msg->wrench.force.y;
    l_ft_force(2) = msg->wrench.force.z;
    l_ft_torque(0) = msg->wrench.torque.x;
    l_ft_torque(1) = msg->wrench.torque.y;
    l_ft_torque(2) = msg->wrench.torque.z;
}

void msgCallbackRightFT(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    r_ft_force(0) = msg->wrench.force.x;
    r_ft_force(1) = msg->wrench.force.y;
    r_ft_force(2) = msg->wrench.force.z;
    r_ft_torque(0) = msg->wrench.torque.x;
    r_ft_torque(1) = msg->wrench.torque.y;
    r_ft_torque(2) = msg->wrench.torque.z;
}
void msgCallbackDualArmCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    int new_mode = (int)msg->data[0];

    if (new_mode == 3) {
        grasp_state = GRASP_APPROACH_JOINT;
        callback = true;      // 아래 while 루프에서 이 트리거로 IK+궤적 계산 시작
        return;
    }

    command_mode = new_mode;
    if (command_mode == 0) {
        for (int i = 0; i < 11; i++) {
            dual_arm_commandp[i] = msg->data[i + 1];
        }
    }
    else {
        for (int i = 0; i < 6; i++) {
            dual_arm_commandx[i] = msg->data[i + 1];
        }
    }
    callback = true;
}
void msgCallbackArucoPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_marker_pose = *msg;
    aruco_pose_received = true;
}
bool transformMarkerToWorld(tf2_ros::Buffer& tfBuffer, Vector3d& out_pos)
{
    if (!aruco_pose_received) return false;

    try {
        geometry_msgs::TransformStamped world_T_cam =
            tfBuffer.lookupTransform("world", aruco_marker_pose.header.frame_id, ros::Time(0));

        geometry_msgs::PoseStamped marker_in_world;
        tf2::doTransform(aruco_marker_pose, marker_in_world, world_T_cam);

        out_pos(0) = marker_in_world.pose.position.x;
        out_pos(1) = marker_in_world.pose.position.y;
        out_pos(2) = marker_in_world.pose.position.z;
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("TF lookup failed: %s", ex.what());
        return false;
    }
}
Vector3d transformForceToWorld(tf2_ros::Buffer& tfBuffer, const Vector3d& force_local, const string& sensor_frame_id)
{
    try {
        geometry_msgs::TransformStamped world_T_sensor =
            tfBuffer.lookupTransform("world", sensor_frame_id, ros::Time(0));

        // 힘은 벡터(vector)이므로 회전만 적용, 평행이동은 무시
        tf2::Quaternion q(
            world_T_sensor.transform.rotation.x,
            world_T_sensor.transform.rotation.y,
            world_T_sensor.transform.rotation.z,
            world_T_sensor.transform.rotation.w);
        tf2::Matrix3x3 R(q);

        tf2::Vector3 f_local(force_local(0), force_local(1), force_local(2));
        tf2::Vector3 f_world = R * f_local;

        return Vector3d(f_world.x(), f_world.y(), f_world.z());
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("Force TF transform failed: %s", ex.what());
        return Vector3d::Zero();
    }
}
void computeGraspPoints(const Vector3d& marker_pos, double box_width, double box_height, double approach_margin, double ee_radius,
                         Vector3d& goalL_approach, Vector3d& goalR_approach,
                         Vector3d& goalL_touch, Vector3d& goalR_touch)
{
    // marker_pos는 "박스 윗면의 중심"이므로, z를 절반 높이만큼 낮춰서 박스의 실제 3D 중심을 구함
    Vector3d box_center = marker_pos - Vector3d(0, 0, box_height / 2.0);

    double grasp_z_offset = -0.05;   // 박스 중심보다 5cm 아래
    
    double half_w = box_width / 2.0;

    goalL_touch = box_center + Vector3d(0, half_w + ee_radius, 0) + Vector3d(0, 0, grasp_z_offset);
    goalR_touch = box_center - Vector3d(0, half_w + ee_radius, 0) + Vector3d(0, 0, grasp_z_offset);

    goalL_approach = box_center + Vector3d(0, half_w + ee_radius + approach_margin, 0);
    goalR_approach = box_center - Vector3d(0, half_w + ee_radius + approach_margin, 0);
}
void updateForceAdmittanceY(double& y_L, double& y_dot_L, double& y_R, double& y_dot_R,
                             double l_force, double r_force)
{
    double F_error_L = l_force - target_grasp_force;
    double F_error_R = r_force - target_grasp_force;

    double y_ddot_L = (F_error_L - D_d * y_dot_L) / M_d;
    double y_ddot_R = (F_error_R - D_d * y_dot_R) / M_d;

    y_dot_L += y_ddot_L * SAMPLING_TIME_TRAJ;
    y_dot_R += y_ddot_R * SAMPLING_TIME_TRAJ;
    y_L += y_dot_L * SAMPLING_TIME_TRAJ;
    y_R += y_dot_R * SAMPLING_TIME_TRAJ;
}
VectorXd computeContactCompensationTorque(pinocchio::Model& model, pinocchio::Data& data,
                                           pinocchio::FrameIndex l_EE, pinocchio::FrameIndex r_EE,
                                           const VectorXd& q_pin,
                                           const Vector3d& l_force_world, const Vector3d& r_force_world)
{
    pinocchio::computeJointJacobians(model, data, q_pin);
    pinocchio::updateFramePlacements(model, data);

    pinocchio::Data::Matrix6x JL(6, model.nv); JL.setZero();
    pinocchio::Data::Matrix6x JR(6, model.nv); JR.setZero();
    pinocchio::getFrameJacobian(model, data, l_EE, pinocchio::LOCAL_WORLD_ALIGNED, JL);
    pinocchio::getFrameJacobian(model, data, r_EE, pinocchio::LOCAL_WORLD_ALIGNED, JR);

    VectorXd F_L(6); F_L.setZero();
    F_L.head<3>() = l_force_world;

    VectorXd F_R(6); F_R.setZero();
    F_R.head<3>() = r_force_world;

    VectorXd tau_contact = JL.transpose() * F_L + JR.transpose() * F_R;
    return tau_contact;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main 함수
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_control_main");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
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
        ros::Publisher dual_armjoint10_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint10_position_controller/command", 100);
        ros::Publisher dual_armjoint11_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint11_position_controller/command", 100);

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
        ros::Publisher dual_armjoint10_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint10_effort_controller/command", 100);
        ros::Publisher dual_armjoint11_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint11_effort_controller/command", 100);
    
    #endif

    // ROS 서브스크라이버 설정 (조인트 상태 토픽)
    ros::Subscriber sub_head_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackHeadJointState);
    ros::Subscriber sub_waist_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackWaistArmJointState);
    ros::Subscriber sub_left_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackLeftArmJointState);
    ros::Subscriber sub_right_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackRightArmJointState);
    ros::Subscriber sub_dual_arm_cmd = nh.subscribe("/dual_arm/DualArmCmd_sim", 100, msgCallbackDualArmCmd);
    ros::Subscriber sub_aruco_pose = nh.subscribe("/aruco_single/pose", 10, msgCallbackArucoPose);
    ros::Subscriber sub_l_ft = nh.subscribe("/dual_arm/l_wrist_ft", 100, msgCallbackLeftFT);
    ros::Subscriber sub_r_ft = nh.subscribe("/dual_arm/r_wrist_ft", 100, msgCallbackRightFT);
   
    ros::Rate loop_rate(1000);
    ros::spinOnce();
    
    // 조인트 명령 메시지 객체 선언
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg;
    std_msgs::Float64 head_yaw_joint_msg, head_pitch_joint_msg;

    // 고정된 URDF 파일 경로 사용
    string urdf_filename = "/home/kimminsu/catkin_ws/src/dual_arm/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // ===== 추가: EE 프레임 ID는 불변이므로 루프 밖에서 한 번만 구함 =====
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < model.njoints; i++) {
        cout << "joint " << i << ": " << model.names[i] << endl;
    }
    while(ros::ok())
    {
        /*
        Vector3d marker_world_pos;
        
        if (transformMarkerToWorld(tfBuffer, marker_world_pos)) {
            marker_pos_world = marker_world_pos;
            cout << "marker_world: x=" << marker_pos_world(0) 
                << " y=" << marker_pos_world(1) 
                << " z=" << marker_pos_world(2) << endl;
        }
        if (transformMarkerToWorld(tfBuffer, marker_world_pos)) {
            marker_pos_world = marker_world_pos;
            double box_width = 0.3;
            double approach_margin = 0.05;
            double EE_radius = 0.0325;
            Vector3d goalL_approach, goalR_approach, goalL_touch, goalR_touch;
            computeGraspPoints(marker_pos_world, box_width, approach_margin, EE_radius,
                                goalL_approach, goalR_approach, goalL_touch, goalR_touch);
            cout << "goalL_approach: " << goalL_approach.transpose()
                << " / goalR_approach: " << goalR_approach.transpose() << endl;
        }
        */

        //cout << "L force: " << l_ft_force.transpose() << " / R force: " << r_ft_force.transpose() << endl;
        Vector3d l_ft_force_world = transformForceToWorld(tfBuffer, l_ft_force, "L_EE");
        Vector3d r_ft_force_world = transformForceToWorld(tfBuffer, r_ft_force, "R_EE");
        //cout << "L force(world): " << l_ft_force_world.transpose() << " / R force(world): " << r_ft_force_world.transpose() << endl;

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
        
        dual_arm_jointp[9]  = head_jointp[0];
        dual_arm_jointv[9]  = head_jointv[0];
        dual_arm_jointp[10] = head_jointp[1];
        dual_arm_jointv[10] = head_jointv[1];

        for (int i = 0; i < DoF; i++){
            dual_arm_jointv_lpf[i] = dualarm.LowPassFilter(dual_arm_jointv[i], dual_arm_jointv_before[i], 10);
            dual_arm_jointv_before[i] = dual_arm_jointv_lpf[i];
        }

        // dual_arm_initp[0] = waist_jointp[0];
        // for (int i = 0; i < 4; i++) {
        //     dual_arm_initp[i + 1] = left_arm_jointp[i];
        //     dual_arm_initp[i + 5] = right_arm_jointp[i];
        // }
        /*
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

        dual_arm_jointp_vec(9)  = head_jointp[0];
        dual_arm_jointv_vec(9)  = head_jointv[0];
        dual_arm_jointp_vec(10) = head_jointp[1];
        dual_arm_jointv_vec(10) = head_jointv[1];
        */

        dual_arm_jointp_vec = convertToPinocchioOrder(dual_arm_jointp);
        dual_arm_jointv_vec = convertToPinocchioOrder(dual_arm_jointv);

        dual_arm_jointv_lpf_vec(0) = waist_jointv[0];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_lpf_vec(i + 1) = left_arm_jointv[i];
            dual_arm_jointv_lpf_vec(i + 5) = right_arm_jointv[i];
        }


        // ================================================================
        // GRASP_CONTACT_WAIT / GRASP_HOLDING: 매 스텝 실시간 힘 제어
        // ================================================================
        if (grasp_state == GRASP_CONTACT_WAIT || grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE) {
            Vector3d l_force_w = transformForceToWorld(tfBuffer, l_ft_force, "L_EE");
            Vector3d r_force_w = transformForceToWorld(tfBuffer, r_ft_force, "R_EE");
            l_contact_force = l_force_w(1);
            r_contact_force = -r_force_w(1);

            double l_force_z_raw = l_force_w(2) + EE_weight;
            double r_force_z_raw = r_force_w(2) + EE_weight;
            l_force_z_fast = dualarm.LowPassFilter(l_force_z_raw, l_force_z_fast, 10.0);
            r_force_z_fast = dualarm.LowPassFilter(r_force_z_raw, r_force_z_fast, 10.0);

            static int z_debug_cnt = 0;
            if (z_debug_cnt++ % 50 == 0) {
                cout << "[Z_FORCE] l_force_z=" << l_force_z_fast << " r_force_z=" << r_force_z_fast << endl;
            }

            l_force_x_fast = dualarm.LowPassFilter(l_force_w(0), l_force_x_fast, 10.0);
            r_force_x_fast = dualarm.LowPassFilter(r_force_w(0), r_force_x_fast, 10.0);

            l_force_fast = dualarm.LowPassFilter(l_contact_force, l_force_fast, 10.0);
            r_force_fast = dualarm.LowPassFilter(r_contact_force, r_force_fast, 10.0);
            l_contact_force_filtered = dualarm.LowPassFilter(l_contact_force, l_contact_force_filtered, 10.0);
            r_contact_force_filtered = dualarm.LowPassFilter(r_contact_force, r_contact_force_filtered, 10.0);

            if (grasp_state == GRASP_CONTACT_WAIT) {
                grasp_state = GRASP_HOLDING;
                // ★직접 현재 관절각으로 채움 (dual_arm_initp의 갱신 타이밍 문제 회피)
                double temp_q[DoF];
                temp_q[0] = waist_jointp[0];
                for (int i = 0; i < 4; i++) {
                    temp_q[i + 1] = left_arm_jointp[i];
                    temp_q[i + 5] = right_arm_jointp[i];
                }
                temp_q[9]  = head_jointp[0];
                temp_q[10] = head_jointp[1];
                q_bias_pin = convertToPinocchioOrder(temp_q);

                waist_angle_locked = waist_jointp[0];

                cout << "[GRASP] Starting force control. q_bias=" << q_bias_pin.transpose() << endl;
            }

            if (grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE) {
                // ── y축: PID 어드미턴스 제어 (안정성 K_d + 정확도 K_i) ──
                double F_error_L = l_contact_force_filtered - target_grasp_force;
                double F_error_R = r_contact_force_filtered - target_grasp_force;

                // ★실제 접촉이 확인된 이후에만 적분 활성화 (초기 과도구간 누적 방지)
                if (!integral_active_L && l_contact_force_filtered > 5.0) {
                    integral_active_L = true;
                }
                if (!integral_active_R && r_contact_force_filtered > 5.0) {
                    integral_active_R = true;
                }

                if (integral_active_L) {
                    integral_L += F_error_L * SAMPLING_TIME_TRAJ;
                }
                if (integral_active_R) {
                    integral_R += F_error_R * SAMPLING_TIME_TRAJ;
                }

                double integral_limit = 2.0;
                integral_L = std::max(-integral_limit, std::min(integral_limit, integral_L));
                integral_R = std::max(-integral_limit, std::min(integral_limit, integral_R));

                double y_ddot_L = (F_error_L - D_d * y_dot_L - K_d_adm * y_L + K_i * integral_L) / M_d;
                double y_ddot_R = (F_error_R - D_d * y_dot_R - K_d_adm * y_R + K_i * integral_R) / M_d;

                y_dot_L += y_ddot_L * SAMPLING_TIME_TRAJ;
                y_dot_R += y_ddot_R * SAMPLING_TIME_TRAJ;
                y_L += y_dot_L * SAMPLING_TIME_TRAJ;
                y_R += y_dot_R * SAMPLING_TIME_TRAJ;

                static int y_debug_cnt = 0;
                if (y_debug_cnt++ % 50 == 0) {
                    cout << "[Y_VAL] y_L=" << y_L << " l_force=" << l_contact_force_filtered
                         << " integral_L=" << integral_L
                         << " y_R=" << y_R << " r_force=" << r_contact_force_filtered
                         << " integral_R=" << integral_R << endl;
                }
                // ── x, z축: 복원 시스템 ──
                pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                pinocchio::updateFramePlacements(model, data);
                Vector3d actualL = data.oMf[l_EE].translation();
                Vector3d actualR = data.oMf[r_EE].translation();

                double x_disturb_L = actualL(0) - grasp_goalL_touch(0);
                double x_disturb_R = actualR(0) - grasp_goalR_touch(0);
                double z_target_now_L = (grasp_state==GRASP_LIFT) ? grasp_current_goalL(2) : grasp_goalL_touch(2);
                double z_target_now_R = (grasp_state==GRASP_LIFT) ? grasp_current_goalR(2) : grasp_goalR_touch(2);
                double z_disturb_L = actualL(2) - z_target_now_L;
                double z_disturb_R = actualR(2) - z_target_now_R;

                double x_ddot_L = (-x_disturb_L - D_xz*x_dot_L - K_xz*x_L) / M_xz;
                double z_ddot_L = (-z_disturb_L - D_xz*z_dot_L - K_xz*z_L) / M_xz;
                double x_ddot_R = (-x_disturb_R - D_xz*x_dot_R - K_xz*x_R) / M_xz;
                double z_ddot_R = (-z_disturb_R - D_xz*z_dot_R - K_xz*z_R) / M_xz;

                x_dot_L += x_ddot_L * SAMPLING_TIME_TRAJ;   z_dot_L += z_ddot_L * SAMPLING_TIME_TRAJ;
                x_L += x_dot_L * SAMPLING_TIME_TRAJ;         z_L += z_dot_L * SAMPLING_TIME_TRAJ;
                x_dot_R += x_ddot_R * SAMPLING_TIME_TRAJ;   z_dot_R += z_ddot_R * SAMPLING_TIME_TRAJ;
                x_R += x_dot_R * SAMPLING_TIME_TRAJ;         z_R += z_dot_R * SAMPLING_TIME_TRAJ;

                // x, y는 공통 조립
                grasp_current_goalL(0) = grasp_goalL_touch(0) + x_L;
                grasp_current_goalL(1) = grasp_goalL_touch(1) + y_L;
                grasp_current_goalR(0) = grasp_goalR_touch(0) + x_R;
                grasp_current_goalR(1) = grasp_goalR_touch(1) - y_R;

                if (grasp_state == GRASP_HOLDING) {
                    grasp_current_goalL(2) = grasp_goalL_touch(2) + z_L;
                    grasp_current_goalR(2) = grasp_goalR_touch(2) + z_R;
                    
                    static int hold_stable_cnt = 0;
                    if (std::abs(l_contact_force_filtered - target_grasp_force) < 5.0 &&
                        std::abs(r_contact_force_filtered - target_grasp_force) < 5.0) {
                        hold_stable_cnt++;
                    } else {
                        hold_stable_cnt = 0;
                    }

                    // ★디버그 추가
                    static int lift_check_cnt = 0;
                    if (lift_check_cnt++ % 100 == 0) {
                        cout << "[LIFT_CHECK] l_force=" << l_contact_force_filtered 
                            << " r_force=" << r_contact_force_filtered
                            << " target=" << target_grasp_force
                            << " stable_cnt=" << hold_stable_cnt << endl;
                    }

                    if (hold_stable_cnt > 500) {
                        grasp_state = GRASP_LIFT;
                        grasp_lift_startZ = grasp_current_goalL(2);
                        hold_stable_cnt = 0;
                        cout << "[GRASP] Stable grip confirmed. Starting lift." << endl;
                    }
                }
                else if (grasp_state == GRASP_LIFT) {

                    pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                    pinocchio::updateFramePlacements(model, data);
                    Vector3d actualL = data.oMf[l_EE].translation();
                    Vector3d actualR = data.oMf[r_EE].translation();

                    static double lift_progress = 0.0;
                    double lift_speed = 0.001;
                    double z_step = lift_speed * SAMPLING_TIME_TRAJ;

                    double actual_z_max = std::max(actualL(2), actualR(2));
                    double z_align_diff = std::abs(actualL(2) - actualR(2));
                    double z_align_threshold = 0.01;   // 2cm, 튜닝 필요

                    double target_z;
                    if (z_align_diff > z_align_threshold) {
                        // 정렬 오차가 크면 전진하지 않고, 느린 쪽이 따라잡을 시간을 줌
                        target_z = std::max(actualL(2), actualR(2));
                    } else {
                        // 정렬이 충분히 맞으면 전진
                        target_z = std::max(actualL(2), actualR(2)) + z_step;
                        target_z = std::min(target_z, grasp_lift_startZ + lift_height);
                    }

                    grasp_current_goalL(2) = target_z;
                    grasp_current_goalR(2) = target_z;

                    if (target_z >= grasp_lift_startZ + lift_height - 0.001) {
                        if (!z_force_locked) {
                            l_force_z_locked = l_force_z_fast;
                            r_force_z_locked = r_force_z_fast;
                            z_force_locked = true;
                            waist_rotate_target = M_PI / 2.0;   // 90도 (왼쪽 방향, 부호는 아래 확인 필요)
                            cout << "[GRASP] Lift complete. z-force locked. Starting rotation." << endl;
                        }
                        grasp_state = GRASP_ROTATE;
                    }
                }

                else if (grasp_state == GRASP_ROTATE) {
                    // z는 최종 높이 유지
                    grasp_current_goalL(2) = grasp_lift_startZ + lift_height;
                    grasp_current_goalR(2) = grasp_lift_startZ + lift_height;
                
                }

                // ── IK, PD ──
                q_ik_seed = convertToPinocchioOrder(dual_arm_initp);
                VectorXd q_force_target;
                bool freeze_waist_now = (grasp_state == GRASP_LIFT || grasp_state == GRASP_HOLDING);
                dualarm.SolveIK_Position(model, data, l_EE, r_EE,
                                        grasp_current_goalL, grasp_current_goalR,
                                        q_ik_seed, q_force_target,
                                        q_bias_pin, k_null, freeze_waist_now);
                VectorXd q_force_target_our = convertFromPinocchioOrder(q_force_target);
                for (int i = 0; i < DoF; i++) {
                    dual_arm_targetp[i] = q_force_target_our(i);
                    dual_arm_targetv[i] = 0.0;
                }

                if (grasp_state == GRASP_ROTATE) {
                    static double waist_progress = 0.0;
                    double waist_speed = 0.01;   // rad/s
                    waist_progress = std::min(waist_progress + waist_speed*SAMPLING_TIME_TRAJ, waist_rotate_target);
                    dual_arm_targetp[0] = waist_progress;

                    // ★허리 회전각만큼 x,y 목표도 회전 변환
                    double theta = dual_arm_targetp[0];   // 현재 허리 목표각
                    double cos_t = cos(theta), sin_t = sin(theta);

                    double origL_x = grasp_goalL_touch(0), origL_y = grasp_goalL_touch(1);
                    grasp_current_goalL(0) = cos_t*origL_x - sin_t*origL_y;
                    grasp_current_goalL(1) = sin_t*origL_x + cos_t*origL_y;

                    double origR_x = grasp_goalR_touch(0), origR_y = grasp_goalR_touch(1);
                    grasp_current_goalR(0) = cos_t*origR_x - sin_t*origR_y;
                    grasp_current_goalR(1) = sin_t*origR_x + cos_t*origR_y;

                    if (waist_progress >= waist_rotate_target - 0.01) {
                        cout << "[GRASP] Rotation complete." << endl;
                    }
                }

                dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);

                // ★디버그 로그 추가
                static int pd_debug_cnt = 0;
                if (pd_debug_cnt++ % 50 == 0) {
                    cout << "[PD_ACC] elbow_L=" << PD_acc[4] << " shoulder_L(sp)=" << PD_acc[1] 
                         << " shoulder_L(sy)=" << PD_acc[3] << endl;
                }

                for (int i = 0; i < DoF; i++) {
                    dual_arm_targeta_vec(i) = PD_acc[i];
                }
            }

            dual_arm_initp[0] = waist_jointp[0];
            for (int i = 0; i < 4; i++) {
                dual_arm_initp[i + 1] = left_arm_jointp[i];
                dual_arm_initp[i + 5] = right_arm_jointp[i];
            }
            for (int i = 0; i < 2; i++) {
                dual_arm_initp[i + 9] = head_jointp[i];
            }
            callback = false;
        }

        else if(callback == true) {
            // 현재 관절각을 initp(궤적 시작점) 및 IK 시드로 저장
            dual_arm_initp[0] = waist_jointp[0];
            for (int i = 0; i < 4; i++) {
                dual_arm_initp[i + 1] = left_arm_jointp[i];
                dual_arm_initp[i + 5] = right_arm_jointp[i];
            }
            dual_arm_initp[9]  = head_jointp[0];   // yaw
            dual_arm_initp[10] = head_jointp[1];   // pitch

            //for (int i = 0; i < DoF; i++) q_ik_seed(i) = dual_arm_initp[i];
            
            q_ik_seed = convertToPinocchioOrder(dual_arm_initp);

            // ★추가: 파지 시퀀스 진입 시 별도 처리
            if (grasp_state == GRASP_APPROACH_JOINT) {
                y_L = 0.0; y_dot_L = 0.0;
                y_R = 0.0; y_dot_R = 0.0;
                x_L = 0.0; x_dot_L = 0.0;
                z_L = 0.0; z_dot_L = 0.0;
                x_R = 0.0; x_dot_R = 0.0;
                z_R = 0.0; z_dot_R = 0.0;
                integral_L = 0.0;   
                integral_R = 0.0;
                integral_active_L = false;   
                integral_active_R = false; 
                grasp_lift_startZ = 0.0;
                z_force_locked = false;
                l_force_z_locked = 0.0;
                r_force_z_locked = 0.0;
                // 현재 관절각 저장 (IK 시드 + 궤적 시작점)
                dual_arm_initp[0] = waist_jointp[0];
                for (int i = 0; i < 4; i++) {
                    dual_arm_initp[i + 1] = left_arm_jointp[i];
                    dual_arm_initp[i + 5] = right_arm_jointp[i];
                }
                for (int i = 0; i < 2; i++) {
                    dual_arm_initp[i + 9] = head_jointp[i];
                }
                
                //for (int i = 0; i < DoF; i++) q_ik_seed(i) = dual_arm_initp[i];

                q_ik_seed = convertToPinocchioOrder(dual_arm_initp);

                // 현재 카메라로 본 마커 위치로 파지점 재계산
                Vector3d marker_pos;
                bool marker_ok;

                if (use_hardcoded_box_position) {
                    // ================================================================
                    // ★임시 디버그: world 파일의 실제 박스 중심 좌표를 그대로 사용
                    //   world 파일 설정: <pose>0.5 0 1.15 0 0 0</pose>
                    //   ※ 이건 "박스 3D 중심"이지 marker_pos(윗면 중심)가 아님에 주의
                    // ================================================================
                    double box_height_temp = 0.3;
                    Vector3d box_center_hardcoded(0.5, 0.0, 1.15);
                    marker_pos = box_center_hardcoded + Vector3d(0, 0, box_height_temp / 2.0);
                    marker_ok = true;
                }
                else {
                    marker_ok = transformMarkerToWorld(tfBuffer, marker_pos);
                }

                if (marker_ok) {
                    double box_width = 0.3;
                    double box_height = 0.3;
                    double approach_margin = 0.05;
                    double EE_radius = 0.0325;
                    computeGraspPoints(marker_pos, box_width, box_height, approach_margin, EE_radius,
                                        grasp_goalL_approach, grasp_goalR_approach,
                                        grasp_goalL_touch, grasp_goalR_touch);
                    
                    // 디버그 출력 
                    cout << "[GRASP] marker_pos: " << marker_pos.transpose() << endl;
                    cout << "[GRASP] goalL_approach: " << grasp_goalL_approach.transpose()
                         << " / goalR_approach: " << grasp_goalR_approach.transpose() << endl;
                    
                    // approach point에 대한 IK 풀기
                    VectorXd q_target;
                    dualarm.SolveIK_Position(model, data, l_EE, r_EE,
                                            grasp_goalL_approach, grasp_goalR_approach,
                                            q_ik_seed, q_target);
                    
                    pinocchio::forwardKinematics(model, data, q_target);
                    pinocchio::updateFramePlacements(model, data);
                    Vector3d actualL = data.oMf[l_EE].translation();
                    Vector3d actualR = data.oMf[r_EE].translation();
                    cout << "[GRASP] IK actual L: " << actualL.transpose() << " (target: " << grasp_goalL_approach.transpose() << ")" << endl;
                    cout << "[GRASP] IK actual R: " << actualR.transpose() << " (target: " << grasp_goalR_approach.transpose() << ")" << endl;

                    VectorXd q_target_our = convertFromPinocchioOrder(q_target);
                    cout << "[GRASP] q_ik_seed: " << q_ik_seed.transpose() << endl;
                    cout << "[GRASP] q_target(IK result): " << q_target_our.transpose() << endl;
                    
                    // IK 결과를 관절각 목표로 변환
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_commandp[i] = q_target_our(i);
                    }

                    // 관절공간 quintic 궤적 생성 (mode1과 동일한 방식)
                    dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp,
                        dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);

                    traj_cnt = 0;
                }
                else {
                    cout << "[GRASP] marker not visible, aborting grasp sequence." << endl;
                    grasp_state = GRASP_IDLE;
                }
            }

            else if (grasp_state == GRASP_APPROACH_LINE) {
                // 현재 관절각으로 시작점(EE 위치) 계산
                dual_arm_initp[0] = waist_jointp[0];
                for (int i = 0; i < 4; i++) {
                    dual_arm_initp[i + 1] = left_arm_jointp[i];
                    dual_arm_initp[i + 5] = right_arm_jointp[i];
                }
                for (int i = 0; i < 2; i++) {
                    dual_arm_initp[i + 9] = head_jointp[i];
                }
                q_ik_seed = convertToPinocchioOrder(dual_arm_initp);

                dual_arm_jointp_vec = convertToPinocchioOrder(dual_arm_initp);
                pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                pinocchio::updateFramePlacements(model, data);
                Vector3d startL = data.oMf[l_EE].translation();
                Vector3d startR = data.oMf[r_EE].translation();

                // touch point까지 직선 궤적 생성
                dualarm.CartesianLineTrajectory(startL, grasp_goalL_touch, startR, grasp_goalR_touch,
                    dual_arm_cart_pos_trajectory, dual_arm_cart_vel_trajectory, dual_arm_cart_acc_trajectory);

                int steps = dual_arm_cart_pos_trajectory.rows();
                dual_arm_jointp_trajectory.resize(steps, DoF);
                dual_arm_jointv_trajectory.resize(steps, DoF);
                dual_arm_jointa_trajectory.resize(steps, DoF);

                VectorXd seed = q_ik_seed;
                for (int k = 0; k < steps; k++) {
                    Vector3d pL(dual_arm_cart_pos_trajectory(k,0), dual_arm_cart_pos_trajectory(k,1), dual_arm_cart_pos_trajectory(k,2));
                    Vector3d pR(dual_arm_cart_pos_trajectory(k,3), dual_arm_cart_pos_trajectory(k,4), dual_arm_cart_pos_trajectory(k,5));

                    VectorXd q_k;
                    dualarm.SolveIK_Position(model, data, l_EE, r_EE, pL, pR, seed, q_k);
                    VectorXd q_k_our = convertFromPinocchioOrder(q_k);
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointp_trajectory(k,i) = q_k_our(i);
                    }
                    seed = q_k;
                }

                dual_arm_jointv_trajectory.setZero();
                for (int k = 1; k < steps - 1; k++) {
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointv_trajectory(k,i) =
                            (dual_arm_jointp_trajectory(k+1,i) - dual_arm_jointp_trajectory(k-1,i)) / (2.0*SAMPLING_TIME_TRAJ);
                    }
                }
                for (int i = 0; i < DoF; i++) {
                    dual_arm_jointv_trajectory(0,i) =
                        (dual_arm_jointp_trajectory(1,i) - dual_arm_jointp_trajectory(0,i)) / SAMPLING_TIME_TRAJ;
                    dual_arm_jointv_trajectory(steps-1,i) =
                        (dual_arm_jointp_trajectory(steps-1,i) - dual_arm_jointp_trajectory(steps-2,i)) / SAMPLING_TIME_TRAJ;
                }

                dual_arm_jointa_trajectory.setZero();
                for (int k = 1; k < steps - 1; k++) {
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointa_trajectory(k,i) =
                            (dual_arm_jointv_trajectory(k+1,i) - dual_arm_jointv_trajectory(k-1,i)) / (2.0*SAMPLING_TIME_TRAJ);
                    }
                }

                traj_cnt = 0;
                cout << "[GRASP] Approach line trajectory generated." << endl;
            }

            // ===== 모드 0: modeling (기존 방식, 관절각 직접) =====
            else if (command_mode == 0) {
                dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp,
                    dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
            }

            // ===== 모드 1: joint sim (위치 IK 1회 → 관절공간 5차 궤적) =====
            else if (command_mode == 1) {
                Vector3d tL(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);
                Vector3d tR(dual_arm_commandx[3], dual_arm_commandx[4], dual_arm_commandx[5]);

                dualarm.SolveIK_Position(model, data, l_EE, r_EE, tL, tR, q_ik_seed, q_ik_result);
                VectorXd q_ik_result_our = convertFromPinocchioOrder(q_ik_result);
                for (int i = 0; i < DoF; i++) dual_arm_commandp[i] = q_ik_result_our(i);

                dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp,
                    dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
            }

            // ===== 모드 2: cartesian sim (직교 직선 → 매 스텝 IK → 관절각 궤적) =====
            else if (command_mode == 2) {
                /*
                for (int i = 0; i < DoF; i++) {
                    dual_arm_jointp_vec(i) = dual_arm_initp[i];
                }
                */
                dual_arm_jointp_vec = convertToPinocchioOrder(dual_arm_initp);

                pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                pinocchio::updateFramePlacements(model, data);
                Vector3d startL = data.oMf[l_EE].translation();
                Vector3d startR = data.oMf[r_EE].translation();

                Vector3d goalL(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);
                Vector3d goalR(dual_arm_commandx[3], dual_arm_commandx[4], dual_arm_commandx[5]);

                // 직교 직선 경로 생성 (위치만 사용, vel/acc는 CartesianLineTrajectory 시그니처상 넘겨받지만 여기선 안 씀)
                dualarm.CartesianLineTrajectory(startL, goalL, startR, goalR,
                    dual_arm_cart_pos_trajectory, dual_arm_cart_vel_trajectory, dual_arm_cart_acc_trajectory);

                int steps = dual_arm_cart_pos_trajectory.rows();
                dual_arm_jointp_trajectory.resize(steps, DoF);
                dual_arm_jointv_trajectory.resize(steps, DoF);
                dual_arm_jointa_trajectory.resize(steps, DoF);

                // === 1단계: 직선 경로의 각 점마다 IK를 풀어서 위치만 먼저 전부 채움 ===
                VectorXd seed = q_ik_seed;
                for (int k = 0; k < steps; k++) {
                    Vector3d pL(dual_arm_cart_pos_trajectory(k,0), dual_arm_cart_pos_trajectory(k,1), dual_arm_cart_pos_trajectory(k,2));
                    Vector3d pR(dual_arm_cart_pos_trajectory(k,3), dual_arm_cart_pos_trajectory(k,4), dual_arm_cart_pos_trajectory(k,5));

                    VectorXd q_k;
                    dualarm.SolveIK_Position(model, data, l_EE, r_EE, pL, pR, seed, q_k);
                    VectorXd q_k_our = convertFromPinocchioOrder(q_k);
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointp_trajectory(k,i) = q_k_our(i);
                    }
                    seed = q_k;  // 다음 점 시드 갱신 (연속성)
                }

                // === 2단계: 위치 시퀀스를 중심차분으로 미분해서 속도 계산 ===
                dual_arm_jointv_trajectory.setZero();
                for (int k = 1; k < steps - 1; k++) {
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointv_trajectory(k,i) =
                            (dual_arm_jointp_trajectory(k+1,i) - dual_arm_jointp_trajectory(k-1,i)) / (2.0*SAMPLING_TIME_TRAJ);
                    }
                }
                // 양 끝단(k=0, k=steps-1)은 전진/후진 차분으로 처리
                for (int i = 0; i < DoF; i++) {
                    dual_arm_jointv_trajectory(0,i) =
                        (dual_arm_jointp_trajectory(1,i) - dual_arm_jointp_trajectory(0,i)) / SAMPLING_TIME_TRAJ;
                    dual_arm_jointv_trajectory(steps-1,i) =
                        (dual_arm_jointp_trajectory(steps-1,i) - dual_arm_jointp_trajectory(steps-2,i)) / SAMPLING_TIME_TRAJ;
                }

                // === 3단계: 속도 시퀀스를 중심차분으로 다시 미분해서 가속도 계산 ===
                dual_arm_jointa_trajectory.setZero();
                for (int k = 1; k < steps - 1; k++) {
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointa_trajectory(k,i) =
                            (dual_arm_jointv_trajectory(k+1,i) - dual_arm_jointv_trajectory(k-1,i)) / (2.0*SAMPLING_TIME_TRAJ);
                    }
                }
            }

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
            if (grasp_state == GRASP_APPROACH_JOINT) {
                cout << "[GRASP] Approach (joint space) complete." << endl;
                grasp_state = GRASP_APPROACH_LINE;   // ★변경: IDLE 대신 다음 단계로
                callback = true;                      // ★추가: 다음 루프에서 GRASP_APPROACH_LINE 궤적 생성 트리거
            }
            else if (grasp_state == GRASP_APPROACH_LINE) {
                cout << "[GRASP] Approach (line) complete. Entering contact wait." << endl;
                grasp_state = GRASP_CONTACT_WAIT;
                grasp_current_goalL = grasp_goalL_touch;   // 힘 제어 시작점 = touch point
                grasp_current_goalR = grasp_goalR_touch;
            }
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
        /*
        for (int i = 0; i < DoF; ++i) {
            dual_arm_targetp_vec(i) = dual_arm_targetp[i];
        }
        */
        dual_arm_targetp_vec = convertToPinocchioOrder(dual_arm_targetp);
        
        // dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_jointv, PD_torque);
        //gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        //dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_vec, dual_arm_targeta_vec);

        // dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_jointv_lpf, PD_torque);
        // gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        // dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_lpf_vec, dual_arm_targeta_vec);
        
        VectorXd dual_arm_targeta_vec_pin = convertToPinocchioOrder(dual_arm_targeta_vec);
        gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_vec, dual_arm_targeta_vec_pin);

        if (grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE) {
            
            Vector3d l_force_w_filtered = Vector3d::Zero();
            Vector3d r_force_w_filtered = Vector3d::Zero();
            l_force_w_filtered(1) = -l_force_fast;
            r_force_w_filtered(1) = r_force_fast;

            l_force_w_filtered(0) = l_force_x_fast;   
            r_force_w_filtered(0) = r_force_x_fast;

            if (grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE) {
                if (z_force_locked) {
                    //l_force_w_filtered(2) = -l_force_z_locked;   // 고정값 사용
                    //r_force_w_filtered(2) = -r_force_z_locked;

                    l_force_w_filtered(2) = -l_force_z_fast;    
                    r_force_w_filtered(2) = -r_force_z_fast;
                } else {
                    l_force_w_filtered(2) = -l_force_z_fast;      // 이동 중엔 실측값
                    r_force_w_filtered(2) = -r_force_z_fast;
                }
            }

            VectorXd tau_contact = computeContactCompensationTorque(model, data, l_EE, r_EE,
                                                                    dual_arm_jointp_vec,
                                                                    l_force_w_filtered, r_force_w_filtered);

            static int tau_debug_cnt = 0;
            if (tau_debug_cnt++ % 50 == 0) {
                cout << "[TAU_CONTACT] " << tau_contact.transpose() << " | l_force_fast=" << l_force_fast << " | r_force_fast=" << r_force_fast << endl;
            }                                                       
                                                                    
            dynamic_torque += tau_contact;
        }

        VectorXd dynamic_torque_our = convertFromPinocchioOrder(dynamic_torque);
        for (int i = 0; i < DoF; i++) {
            target_torque[i] = dynamic_torque_our(i);
        }
        
        /*
        for (int i = 0; i < DoF; i++) {
            target_torque[i] = dynamic_torque(i);
        }
        */
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
            head_yaw_joint_msg.data         = dual_arm_targetp[9];
            head_pitch_joint_msg.data       = dual_arm_targetp[10];

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
            head_yaw_joint_msg.data         = target_torque[9];
            head_pitch_joint_msg.data       = target_torque[10];


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
        dual_armjoint10_pub.publish(head_yaw_joint_msg);
        dual_armjoint11_pub.publish(head_pitch_joint_msg);

        
        cout << fixed << setprecision(4);

        // cout << dual_arm_jointp[0]*rad2deg << endl;
        // cout << dual_arm_jointp[1]*rad2deg << "   " << dual_arm_jointp[5]*rad2deg << endl;
        // cout << dual_arm_jointp[2]*rad2deg << "   " << dual_arm_jointp[6]*rad2deg << endl;
        // cout << dual_arm_jointp[3]*rad2deg << "   " << dual_arm_jointp[7]*rad2deg << endl;
        // cout << dual_arm_jointp[4]*rad2deg << "   " << dual_arm_jointp[8]*rad2deg << endl;
        
        /*
        cout << "=== Left End-Effector Pose ===" << endl;
        cout << "Position: " << data.oMf[l_EE].translation().transpose() << endl;
        cout << "Orientation (Rotation Matrix):\n" << data.oMf[l_EE].rotation() << endl;
        
        cout << "=== Right End-Effector Pose ===" << endl;
        cout << "Position: " << data.oMf[r_EE].translation().transpose() << endl;
        cout << "Orientation (Rotation Matrix):\n" << data.oMf[r_EE].rotation() << endl;     
        */

        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}