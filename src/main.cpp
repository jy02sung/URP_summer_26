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
        if (grasp_state == GRASP_CONTACT_WAIT || grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE) {

            // ★공통 theta_now 계산 (모든 곳에서 이 값을 재사용)

            // ★HOLDING/LIFT 동안 theta_now_common은 실측 드리프트(waist_jointp-waist_ref_angle)를 그대로 따라감.
            //   ROTATE 진입 순간 그 실측 드리프트를 그대로 이어받은 뒤(waist_ref_angle_at_rotate_start-waist_ref_angle),
            //   그 위에 "계획된" 회전량(waist_theta_goal)을 더함 → 전환 시점에 값이 뚝 끊기지 않고 연속.
            if (grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE) {
                theta_now_common = waist_theta_goal + (waist_ref_angle_at_rotate_start - waist_ref_angle);
            } else {
                theta_now_common = waist_jointp[0] - waist_ref_angle;
            }

            // ★힘 센서/반력 보상 전용 실측각. waist_ref_angle 하나만 기준으로 삼아 상태 전환과 무관하게 항상 연속.
            theta_now_actual = waist_jointp[0] - waist_ref_angle;

            Vector3d l_force_w = transformForceToWorld(tfBuffer, l_ft_force, "L_EE");
            Vector3d r_force_w = transformForceToWorld(tfBuffer, r_ft_force, "R_EE");

            // ★몸(허리) 기준 좌표계로 변환 (x, y만 필요, z는 회전과 무관) — 실측각 사용
            Vector3d l_force_body = InverseRotateZ(l_force_w, theta_now_actual);
            Vector3d r_force_body = InverseRotateZ(r_force_w, theta_now_actual);

            l_contact_force = l_force_body(1);
            r_contact_force = -r_force_body(1);
            Vector3d l_force_body_full = InverseRotateZ(l_force_w, theta_now_actual);
            Vector3d r_force_body_full = InverseRotateZ(r_force_w, theta_now_actual);

            double l_force_z_raw = l_force_body_full(2) + EE_weight;
            double r_force_z_raw = r_force_body_full(2) + EE_weight;
            l_force_z_fast = dualarm.LowPassFilter(l_force_z_raw, l_force_z_fast, 10.0);
            r_force_z_fast = dualarm.LowPassFilter(r_force_z_raw, r_force_z_fast, 10.0);
            static int z_debug_cnt = 0;
            if (z_debug_cnt++ % 50 == 0) {
                cout << "[Z_FORCE] l_force_z=" << l_force_z_fast << " r_force_z=" << r_force_z_fast << endl;
                cout << "[Z_RAW] l_ft_force(local)=" << l_ft_force.transpose()
                     << " r_ft_force(local)=" << r_ft_force.transpose()
                     << " | l_force_w=" << l_force_w.transpose()
                     << " r_force_w=" << r_force_w.transpose() << endl;
            }

            l_force_x_fast = dualarm.LowPassFilter(l_force_body(0), l_force_x_fast, 10.0);
            r_force_x_fast = dualarm.LowPassFilter(r_force_body(0), r_force_x_fast, 10.0);
            l_force_fast = dualarm.LowPassFilter(l_contact_force, l_force_fast, 10.0);
            r_force_fast = dualarm.LowPassFilter(r_contact_force, r_force_fast, 10.0);
            l_contact_force_filtered = dualarm.LowPassFilter(l_contact_force, l_contact_force_filtered, 10.0);
            r_contact_force_filtered = dualarm.LowPassFilter(r_contact_force, r_contact_force_filtered, 10.0);

            if (grasp_state == GRASP_CONTACT_WAIT) {
                grasp_state = GRASP_HOLDING;

                waist_ref_angle = waist_jointp[0];

                box_offset_from_axis = box_center_world - waist_axis_world;

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

            if (grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE) {
                // ── y축: PID 어드미턴스 제어 (안정성 K_d + 정확도 K_i) ──
                // ★RELEASE 중에는 일부러 힘을 놓는 중이라, 어드미턴스가 다시 조이려 하면 안 됨 → y_L/y_R 갱신 자체를 멈춤(동결)
                if (grasp_state != GRASP_RELEASE) {
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
                }
                // ── 몸 기준 좌표계로 목표 위치 계산 ──
                double theta_actual = waist_jointp[0] - waist_ref_angle;
                double theta_now_rotate = waist_jointp[0] - waist_ref_angle_at_rotate_start;

                if (grasp_state == GRASP_ROTATE) {
                    // 현재 EE 실제 위치 확인 (FK)
                    pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                    pinocchio::updateFramePlacements(model, data);
                    Vector3d actualL_check = data.oMf[l_EE].translation();
                    Vector3d actualR_check = data.oMf[r_EE].translation();

                    double ee_L_error = (actualL_check - grasp_current_goalL).norm();
                    double ee_R_error = (actualR_check - grasp_current_goalR).norm();
                    double ee_L_error_2 = std::abs(actualL_check(2) - grasp_current_goalL(2));
                    double ee_R_error_2 = std::abs(actualR_check(2) - grasp_current_goalR(2));
                    double ee_error_3 = std::abs(actualR_check(2) - actualL_check(2));
                    double ee_reach_tolerance = 0.008;

                    double waist_step = 0.002;
                    double reach_tolerance = 0.002;

                    bool waist_reached = std::abs(theta_now_rotate - waist_theta_goal) < reach_tolerance;
                    bool ee_reached = (ee_L_error < ee_reach_tolerance) && (ee_R_error < ee_reach_tolerance);
                    //&& (ee_L_error_2 < ee_reach_tolerance) && (ee_R_error_2 < ee_reach_tolerance) && (ee_error_3 <= ee_reach_tolerance+0.001)
                    static int rotate_check_cnt = 0;
                    if (waist_reached && ee_reached) {
                        rotate_check_cnt += 1;
                    }
                    else{
                        rotate_check_cnt = 0;
                    }

                    if (rotate_check_cnt>=100){
                        waist_theta_goal = std::min(waist_theta_goal + waist_step, waist_rotate_target);
                        rotate_check_cnt = 0;
                    }
                    // 도달 못했으면 waist_theta_goal 그대로 유지 (대기)

                    // ★디버그: 회전이 왜 안 시작/진행되는지 확인용
                    static int rotate_debug_cnt = 0;
                    if (rotate_debug_cnt++ % 100 == 0) {
                        cout << "[ROTATE_DEBUG] ee_L_error=" << ee_L_error
                             << " ee_R_error=" << ee_R_error
                             << " waist_reached=" << waist_reached
                             << " ee_reached=" << ee_reached
                             << " theta_now_rotate=" << theta_now_rotate
                             << " waist_theta_goal=" << waist_theta_goal
                             << " waist_jointp=" << waist_jointp[0] << endl;
                    }
                }

                double half_w = box_width / 2.0;

                if (grasp_state == GRASP_ROTATE && waist_theta_goal >= waist_rotate_target - 0.01) {
                    static bool rotate_logged = false;
                    if (!rotate_logged) {
                        cout << "[GRASP] Rotation complete. Starting place." << endl;
                        rotate_logged = true;
                        box_center_z_place_start = box_center_world(2);
                        grasp_state = GRASP_PLACE;
                    }
                }

                // 몸 기준(회전 전) touch point를 매 순간 재계산
                Vector3d touch_offset_L(0, half_w + EE_radius, 0);
                Vector3d touch_offset_R(0, -(half_w + EE_radius), 0);
                Vector3d touch_L_world = waist_axis_world + RotateZ(box_offset_from_axis + touch_offset_L, theta_now_common);
                Vector3d touch_R_world = waist_axis_world + RotateZ(box_offset_from_axis + touch_offset_R, theta_now_common);

                // ── x, z축: 복원 시스템 (회전된 touch point 기준으로 오차 계산) ──
                pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                pinocchio::updateFramePlacements(model, data);
                Vector3d actualL = data.oMf[l_EE].translation();
                Vector3d actualR = data.oMf[r_EE].translation();

 
                // ★box_center_world(2)는 HOLDING(고정 높이)/LIFT(상승 중 높이)/ROTATE(최종 높이) 모두 항상
                //   "지금 실제로 있어야 할 높이"를 담고 있으므로 상태 분기 없이 그대로 기준으로 씀.
                //   (기존엔 LIFT 외 상태에서 touch_L_world(2)를 썼는데, 이는 box_offset_from_axis에 박힌
                //    "들어올리기 전" 높이라서 ROTATE 진입 후 lift_height만큼 어긋난 기준이 되는 버그였음)
                double z_target_now_L = box_center_world(2);
                double z_target_now_R = box_center_world(2);
 
                Vector3d disturb_L_world(actualL(0) - touch_L_world(0), 0, actualL(2) - z_target_now_L);
                Vector3d disturb_R_world(actualR(0) - touch_R_world(0), 0, actualR(2) - z_target_now_R);

                Vector3d disturb_L_body = InverseRotateZ(disturb_L_world, theta_now_common);
                Vector3d disturb_R_body = InverseRotateZ(disturb_R_world, theta_now_common);

                double x_disturb_L = disturb_L_body(0);
                double z_disturb_L = disturb_L_body(2);
                double x_disturb_R = disturb_R_body(0);
                double z_disturb_R = disturb_R_body(2);

                double x_ddot_L = (-x_disturb_L - D_xz*x_dot_L - K_xz*x_L) / M_xz;
                double z_ddot_L = (-z_disturb_L - D_xz*z_dot_L - K_xz*z_L) / M_xz;
                double x_ddot_R = (-x_disturb_R - D_xz*x_dot_R - K_xz*x_R) / M_xz;
                double z_ddot_R = (-z_disturb_R - D_xz*z_dot_R - K_xz*z_R) / M_xz;

                x_dot_L += x_ddot_L * SAMPLING_TIME_TRAJ;   z_dot_L += z_ddot_L * SAMPLING_TIME_TRAJ;
                x_L += x_dot_L * SAMPLING_TIME_TRAJ;         z_L += z_dot_L * SAMPLING_TIME_TRAJ;
                x_dot_R += x_ddot_R * SAMPLING_TIME_TRAJ;   z_dot_R += z_ddot_R * SAMPLING_TIME_TRAJ;
                x_R += x_dot_R * SAMPLING_TIME_TRAJ;         z_R += z_dot_R * SAMPLING_TIME_TRAJ;

                // ── 최종 목표: 몸 기준 오프셋(x_L,y_L,z_L)을 계산해서 회전 후 조립 ──
                // ★RELEASE 중에는 release_offset만큼 양쪽으로 더 벌어지도록 y에 추가
                double release_y_extra = (grasp_state == GRASP_RELEASE) ? release_offset : 0.0;
                Vector3d body_offset_L(x_L, half_w + EE_radius + y_L + release_y_extra, z_L);
                Vector3d body_offset_R(x_R, -(half_w + EE_radius) - y_R - release_y_extra, z_R);

                // ★HOLDING/LIFT/ROTATE 공통 공식: 박스는 항상 허리축(waist_axis_world) 기준으로 회전.
                //   HOLDING/LIFT는 theta_now_common≈0이라 기존 동작과 동일, ROTATE에서도 힘 피드백(body_offset)이
                //   그대로 반영되면서 박스 가로축이 로봇-박스중심 연결선에 수직으로 유지됨.
                grasp_current_goalL = waist_axis_world + RotateZ(box_offset_from_axis + body_offset_L, theta_now_common);
                grasp_current_goalR = waist_axis_world + RotateZ(box_offset_from_axis + body_offset_R, theta_now_common);

                if (grasp_state == GRASP_HOLDING) {

                    static int hold_stable_cnt = 0;
                    if (std::abs(l_contact_force_filtered - target_grasp_force) < 3.0 &&
                        std::abs(r_contact_force_filtered - target_grasp_force) < 3.0) {
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

                    if (hold_stable_cnt > 100) {
                        grasp_state = GRASP_LIFT;
                        box_center_z_start = box_center_world(2);
                        hold_stable_cnt = 0;
                        cout << "[GRASP] Stable grip confirmed. Starting lift." << endl;
                    }
                }
                else if (grasp_state == GRASP_LIFT) {
                    pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                    pinocchio::updateFramePlacements(model, data);
                    Vector3d actualL = data.oMf[l_EE].translation();
                    Vector3d actualR = data.oMf[r_EE].translation();

                    double lift_speed = 0.01;
                    double z_step = lift_speed * SAMPLING_TIME_TRAJ;

                    double z_align_diff = std::abs(actualL(2) - actualR(2));
                    double z_align_threshold = 0.005;

                    if (z_align_diff <= z_align_threshold) {
                        double check_z = std::max(actualL(2), actualR(2));
                        double new_z = std::min(check_z + z_step, box_center_z_start + lift_height);
                        box_center_world(2) = new_z;
                        grasp_current_goalL(2) = box_center_world(2);
                        grasp_current_goalR(2) = box_center_world(2);
                    }
                    else{
                        grasp_current_goalL(2) = std::max(actualL(2), actualR(2));
                        grasp_current_goalR(2) = std::max(actualL(2), actualR(2));
                        box_center_world(2) = std::max(actualL(2), actualR(2));
                    }
                    // 정렬 안 맞으면 box_center_world(2) 그대로 유지 (전진 정지)

                    if (box_center_world(2) >= box_center_z_start + lift_height - 0.001) {
                        if (!z_force_locked) {
                            l_force_z_locked = l_force_z_fast;
                            r_force_z_locked = r_force_z_fast;
                            z_force_locked = true;
                            waist_rotate_target = 40.0 * deg2rad;   // Waist_joint URDF 한계(±0.87rad≈49.8도) 안쪽으로
                            cout << "[GRASP] Lift complete. z-force locked. Starting rotation." << endl;

                            waist_ref_angle_at_rotate_start = waist_jointp[0];
                            grasp_state = GRASP_ROTATE;

                        }
                        grasp_state = GRASP_ROTATE;
                    }
                }

                else if (grasp_state == GRASP_ROTATE) {
                    // z는 최종 높이 유지 (box_offset_from_axis의 z는 들어올리기 전 높이라 그대로 두면 안 됨 → 덮어씀)
                    // 위치 목표는 항상 기구학 그대로 유지하고, 부족한 지지력은 아래 반력 보상(토크) 쪽에서 채운다.
                    grasp_current_goalL(2) = box_center_world(2);
                    grasp_current_goalR(2) = box_center_world(2);
                }

                else if (grasp_state == GRASP_PLACE) {
                    // ★LIFT와 대칭 구조: 위로 올리는 대신 아래로 내림. z_align_diff 안전장치도 동일하게 적용.
                    pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                    pinocchio::updateFramePlacements(model, data);
                    Vector3d actualL = data.oMf[l_EE].translation();
                    Vector3d actualR = data.oMf[r_EE].translation();

                    double place_speed = 0.01;
                    double z_step = place_speed * SAMPLING_TIME_TRAJ;

                    double z_align_diff = std::abs(actualL(2) - actualR(2));
                    double z_align_threshold = 0.005;

                    if (z_align_diff <= z_align_threshold) {
                        double check_z = std::min(actualL(2), actualR(2));
                        double new_z = std::max(check_z - z_step, box_center_z_place_start - place_height);
                        box_center_world(2) = new_z;
                        grasp_current_goalL(2) = box_center_world(2);
                        grasp_current_goalR(2) = box_center_world(2);
                    }
                    else {
                        grasp_current_goalL(2) = std::min(actualL(2), actualR(2));
                        grasp_current_goalR(2) = std::min(actualL(2), actualR(2));
                        box_center_world(2) = std::min(actualL(2), actualR(2));
                    }
                    // 정렬 안 맞으면 box_center_world(2) 그대로 유지 (하강 정지)

                    // ★박스가 살짝 기울어진 채로 내려가면 아랫면 모서리가 책상에 먼저 닿아서
                    //   목표 높이(box_center_z_place_start - place_height)까지 못 내려갈 수 있음.
                    //   그럴 땐 실제 높이가 더 이상 안 바뀌는 것 자체를 "다 내려갔다"는 신호로 사용.
                    static double last_z_for_stability = 1e9;
                    static int z_stable_cnt = 0;
                    double current_z = std::max(actualL(2), actualR(2));
                    double z_stability_tol = 0.001;   // 1mm
                    if (std::abs(current_z - last_z_for_stability) < z_stability_tol) {
                        z_stable_cnt++;
                    } else {
                        z_stable_cnt = 0;
                    }
                    last_z_for_stability = current_z;
                    bool z_settled = (z_stable_cnt >= 5000);   // 5초(1kHz 루프 기준) 이상 정지

                    if ((box_center_world(2) <= box_center_z_place_start - place_height + 0.001) || z_settled) {
                        static bool place_logged = false;
                        if (!place_logged) {
                            cout << "[GRASP] Place complete (settled=" << z_settled << "). Starting release." << endl;
                            place_logged = true;
                            grasp_state = GRASP_RELEASE;
                        }
                    }
                }

                else if (grasp_state == GRASP_RELEASE) {
                    // z는 놓인 높이 그대로 유지, y만 양쪽으로 벌림
                    grasp_current_goalL(2) = box_center_world(2);
                    grasp_current_goalR(2) = box_center_world(2);

                    // ★처음엔 천천히 빠져서 마찰이 서서히 풀리며 자연스럽게 내려앉게 하고, 이후 점점 빠르게 마무리.
                    //   진행률(0→1)에 비례해 속도를 선형으로 증가시킴.
                    double release_progress = release_offset / release_distance;
                    double current_release_speed = release_speed_start + (release_speed_end - release_speed_start) * release_progress;
                    release_offset = std::min(release_offset + current_release_speed * SAMPLING_TIME_TRAJ, release_distance);

                    if (release_offset >= release_distance - 0.0005) {
                        static bool release_logged = false;
                        if (!release_logged) {
                            cout << "[GRASP] Release complete." << endl;
                            release_logged = true;

                            // ★GRASP_IDLE로 빠지면 힘 제어 블록 밖으로 나가서, 훨씬 예전(APPROACH_LINE)에
                            //   만들어둔 낡은 궤적의 마지막 지점을 다시 목표로 잡아버림(→ 정면으로 홱 돌아가는 원인).
                            //   지금 실제 관절각을 그 "마지막 궤적 지점" 자리에 덮어써서, 해제 직후 자세 그대로 유지되게 함.
                            double temp_q[DoF];
                            temp_q[0] = waist_jointp[0];
                            for (int i = 0; i < 4; i++) {
                                temp_q[i + 1] = left_arm_jointp[i];
                                temp_q[i + 5] = right_arm_jointp[i];
                            }
                            temp_q[9]  = head_jointp[0];
                            temp_q[10] = head_jointp[1];

                            dual_arm_jointp_trajectory.resize(1, DoF);
                            dual_arm_jointv_trajectory.resize(1, DoF);
                            dual_arm_jointa_trajectory.resize(1, DoF);
                            for (int i = 0; i < DoF; i++) {
                                dual_arm_jointp_trajectory(0, i) = temp_q[i];
                                dual_arm_jointv_trajectory(0, i) = 0.0;
                                dual_arm_jointa_trajectory(0, i) = 0.0;
                            }

                            grasp_state = GRASP_IDLE;
                        }
                    }
                }

                // ── IK, PD ──
                q_ik_seed = convertToPinocchioOrder(dual_arm_initp);
                VectorXd q_force_target;
                bool freeze_waist_now = (grasp_state == GRASP_LIFT || grasp_state == GRASP_HOLDING || grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE);

                double waist_lock_value_now = 0.0;
                if (grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE) {
                    waist_lock_value_now = waist_theta_goal + waist_ref_angle_at_rotate_start;   // 몸 기준 목표를 다시 절대 관절각으로 (회전 완료 각도 유지)
                } else if (grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT) {
                    waist_lock_value_now = waist_ref_angle;   // 파지 중엔 원래 각도(보통 0) 유지
                }

                dualarm.SolveIK_Position(model, data, l_EE, r_EE,
                                        grasp_current_goalL, grasp_current_goalR,
                                        q_ik_seed, q_force_target,
                                        q_bias_pin, k_null, freeze_waist_now, waist_lock_value_now);
                VectorXd q_force_target_our = convertFromPinocchioOrder(q_force_target);
                for (int i = 0; i < DoF; i++) {
                    dual_arm_targetp[i] = q_force_target_our(i);
                    dual_arm_targetv[i] = 0.0;
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
                box_center_z_place_start = 0.0;
                release_offset = 0.0;
                waist_target_initialized = false;
                waist_theta_goal = 0.0;
                waist_ref_angle_at_rotate_start = 0.0;
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
                    box_width = 0.3;
                    double box_height = 0.3;
                    double approach_margin = 0.05;
                    EE_radius = 0.0325;
                    
                    // computeGraspPoints 호출 직전/직후에 추가
                    box_center_world = marker_pos - Vector3d(0, 0, box_height/2.0) + Vector3d(0, 0, grasp_z_offset);

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

        if (grasp_state == GRASP_HOLDING || grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE || grasp_state == GRASP_RELEASE) {

            Vector3d l_force_w_filtered = Vector3d::Zero();
            Vector3d r_force_w_filtered = Vector3d::Zero();

            // ★몸 기준으로 계산된 x,y 반력을 다시 월드 좌표로 회전 변환 — 실측각 사용
            Vector3d l_force_body_vec(l_force_x_fast, -l_force_fast, 0);
            Vector3d r_force_body_vec(r_force_x_fast, r_force_fast, 0);
            Vector3d l_force_w_xy = RotateZ(l_force_body_vec, theta_now_actual);
            Vector3d r_force_w_xy = RotateZ(r_force_body_vec, theta_now_actual);

            l_force_w_filtered(0) = l_force_w_xy(0);
            l_force_w_filtered(1) = l_force_w_xy(1);
            r_force_w_filtered(0) = r_force_w_xy(0);
            r_force_w_filtered(1) = r_force_w_xy(1);

            if (grasp_state == GRASP_LIFT || grasp_state == GRASP_ROTATE || grasp_state == GRASP_PLACE) {
                Vector3d l_force_body_z(0, 0, -l_force_z_fast);
                Vector3d r_force_body_z(0, 0, -r_force_z_fast);

                Vector3d l_force_w_z = RotateZ(l_force_body_z, theta_now_actual);
                Vector3d r_force_w_z = RotateZ(r_force_body_z, theta_now_actual);

                l_force_w_filtered(2) = l_force_w_z(2);
                r_force_w_filtered(2) = r_force_w_z(2);
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