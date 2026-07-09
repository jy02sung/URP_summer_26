#include "dual_arm_function.cpp"
#include <ros/package.h> // 패키지 경로 자동 탐색용
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>

bool left_contact = false;
bool right_contact = false;
bool do_aruco_pick = false;
int pick_step = 0; 
double current_squeeze_L = 0.10;
double current_squeeze_R = -0.10;
double head_start_angle = 0.0;

geometry_msgs::PoseStamped latest_aruco_pose_cam; // 카메라 기준 좌표
bool aruco_detected = false; // 마커 감지 플래그

// 왼팔 힘 센서 콜백
void leftBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
    left_contact = (msg->states.size() > 0); 
}

// 오른팔 힘 센서 콜백
void rightBumperCallback(const gazebo_msgs::ContactsState::ConstPtr& msg) {
    right_contact = (msg->states.size() > 0);
}

void arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_aruco_pose_cam = *msg; // 카메라 기준 좌표 저장
    aruco_detected = true;        // 감지 플래그 ON
}

void msgCallbackArucoPick(const std_msgs::Empty::ConstPtr& msg) {
    do_aruco_pick = true;
    pick_step = 1;
    current_squeeze_L = 0.10; // 큐브 중심에서 10cm 떨어진 곳부터 시작
    current_squeeze_R = -0.10;
    ROS_INFO(">>> Aruco Pick Sequence Started!");
}

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
    tf::TransformListener tf_listener; // TF를 들어주는 리스너 객체 생성
    
    ros::Subscriber sub_left_force = nh.subscribe("/dual_arm/left_force", 10, leftBumperCallback);
    ros::Subscriber sub_right_force = nh.subscribe("/dual_arm/right_force", 10, rightBumperCallback);
    
    // 비전 토픽 구독 등록
    ros::Subscriber sub_aruco = nh.subscribe("/aruco_single/pose", 10, arucoPoseCallback);
    ros::Subscriber sub_aruco_pick = nh.subscribe("/dual_arm/ArucoPickCmd", 10, msgCallbackArucoPick);
    

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
        // [최종] 아루코 큐브 파지 시퀀스 상태 관리
        // =========================================================================
        if (do_aruco_pick) {
            // [Phase 0] 머리 숙이기 (팔은 Cartesian 제어로 현재 위치에 완벽 고정!)
            if (pick_step == 1) { 
                Vector3d current_L = data.oMf[l_EE].translation();
                Vector3d current_R = data.oMf[r_EE].translation();
                
                // 팔이 제자리에 머물도록 시작과 끝이 똑같은 '정지 궤적' 생성
                dualarm.CartesianTrajectoryQuintic(current_L, current_L, cartesian_p_trajectory_L, cartesian_v_trajectory_L, cartesian_a_trajectory_L);
                dualarm.CartesianTrajectoryQuintic(current_R, current_R, cartesian_p_trajectory_R, cartesian_v_trajectory_R, cartesian_a_trajectory_R);
                
                head_start_angle = dual_arm_targetp[10]; // 현재 머리 각도 기억
                
                traj_cnt = 0;
                is_cartesian_moving = true; // Joint 모드로 넘어가지 않고 Cartesian 유지!!
                pick_step = 2; 
                ROS_INFO("[Phase 0] Tilting head. Arms held in Cartesian space.");
            }
            // [Phase 0] 머리가 40도까지 다 숙여질 때까지 대기
            else if (pick_step == 2) {
                if (traj_cnt >= cartesian_p_trajectory_L.rows()) {
                    pick_step = 3;
                    ROS_INFO("Head moved. Waiting for ArUco Marker...");
                }
            }
            // [Phase 1] 비전 확인 및 접근(Approach) 궤적 생성
            else if (pick_step == 3) { 
                if (!aruco_detected) {
                    ROS_WARN_THROTTLE(1.0, "Wait: Aruco marker is not detected yet.");
                } 
                else {
                    geometry_msgs::PoseStamped pose_in_base;
                    try {
                        tf_listener.transformPose("world_link", latest_aruco_pose_cam, pose_in_base);
                        
                        double transformed_x = pose_in_base.pose.position.x;
                        double transformed_z = pose_in_base.pose.position.z;
                        ROS_INFO("ArUco Detected! X=%.2f, Z=%.2f", transformed_x, transformed_z);

                        Vector3d current_L = data.oMf[l_EE].translation();
                        Vector3d current_R = data.oMf[r_EE].translation();

                        Vector3d target_L(transformed_x, 0.10, transformed_z);
                        Vector3d target_R(transformed_x, -0.10, transformed_z);
                        
                        dualarm.CartesianTrajectoryQuintic(current_L, target_L, cartesian_p_trajectory_L, cartesian_v_trajectory_L, cartesian_a_trajectory_L);
                        dualarm.CartesianTrajectoryQuintic(current_R, target_R, cartesian_p_trajectory_R, cartesian_v_trajectory_R, cartesian_a_trajectory_R);
                        
                        traj_cnt = 0;
                        is_cartesian_moving = true; 
                        pick_step = 4;
                        ROS_INFO("[Phase 1] Approach Trajectory Generated.");
                    } catch (tf::TransformException &ex) {
                        ROS_ERROR_THROTTLE(1.0, "TF Transform Failed: %s", ex.what());
                    }
                }
            }
            // [Phase 1] 팔이 큐브 옆에 도달할 때까지 대기
            else if (pick_step == 4) { 
                if (traj_cnt >= cartesian_p_trajectory_L.rows()) {
                    pick_step = 5; 
                    ROS_INFO("[Phase 2] Squeezing started.");
                }
            }
            // pick_step == 5 는 아래쪽 조이기 루프에서 처리
            // [Phase 3] 들어올리기(Lift) 궤적 생성
            else if (pick_step == 6) { 
                Vector3d current_L = data.oMf[l_EE].translation();
                Vector3d current_R = data.oMf[r_EE].translation();

                Vector3d target_L(current_L.x(), current_squeeze_L, current_L.z() + 0.15); 
                Vector3d target_R(current_R.x(), current_squeeze_R, current_R.z() + 0.15);
                
                dualarm.CartesianTrajectoryQuintic(current_L, target_L, cartesian_p_trajectory_L, cartesian_v_trajectory_L, cartesian_a_trajectory_L);
                dualarm.CartesianTrajectoryQuintic(current_R, target_R, cartesian_p_trajectory_R, cartesian_v_trajectory_R, cartesian_a_trajectory_R);
                
                traj_cnt = 0;
                is_cartesian_moving = true;
                pick_step = 7;
                ROS_INFO("[Phase 3] Lifting Trajectory Generated.");
            }
            // [Phase 3] 리프팅 대기 및 시퀀스 종료
            else if (pick_step == 7) { 
                if (traj_cnt >= cartesian_p_trajectory_L.rows()) {
                    do_aruco_pick = false; 
                    pick_step = 0;
                    ROS_INFO("Cube is Picked Up! Sequence Fully Completed.");
                }
            }
        }

        // =========================================================================
        // Phase 2: Squeeze (조이기) 실시간 제어 블록
        // =========================================================================
        else if (do_aruco_pick && pick_step == 5) {
            bool L_done = left_contact;
            bool R_done = right_contact;

            if (!L_done) current_squeeze_L -= 0.00005; 
            if (!R_done) current_squeeze_R += 0.00005;

            pinocchio::SE3 target_pose_L = data.oMf[l_EE];
            target_pose_L.translation()(1) = current_squeeze_L; 
            
            pinocchio::SE3 target_pose_R = data.oMf[r_EE];
            target_pose_R.translation()(1) = current_squeeze_R; 

            Eigen::VectorXd q_ik = dual_arm_jointp_vec;
            dualarm.SolveIK_DLS(model, data, l_EE, target_pose_L, q_ik);
            dualarm.SolveIK_DLS(model, data, r_EE, target_pose_R, q_ik);

            for (int i = 0; i < DoF; i++) {
                dual_arm_targetp[i] = q_ik(i);
                dual_arm_targetv[i] = 0.0; 
                dual_arm_targeta_vec(i) = 0.0;
            }

            // 고개 강제 고정 덮어쓰기
            dual_arm_targetp[10] = -40.0 * M_PI / 180.0;

            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
            for (int i = 0; i < DoF; i++){
                dual_arm_targeta_vec(i) += PD_acc[i];
            }

            if (L_done && R_done) {
                ROS_INFO("Cube is Secured! Both arms touched.");
                pick_step = 6;
            }
        }

        // =========================================================================
        // 0번: Cartesian 직선 궤적 추종 (매 틱 IK 계산)
        // =========================================================================
        else if (is_cartesian_moving && traj_cnt < cartesian_p_trajectory_L.rows()) {
            
            pinocchio::SE3 target_pose_L = data.oMf[l_EE]; 
            target_pose_L.translation() << cartesian_p_trajectory_L(traj_cnt, 0), cartesian_p_trajectory_L(traj_cnt, 1), cartesian_p_trajectory_L(traj_cnt, 2);
            
            pinocchio::SE3 target_pose_R = data.oMf[r_EE];
            target_pose_R.translation() << cartesian_p_trajectory_R(traj_cnt, 0), cartesian_p_trajectory_R(traj_cnt, 1), cartesian_p_trajectory_R(traj_cnt, 2);

            Eigen::VectorXd q_ik = dual_arm_jointp_vec;
            
            dualarm.SolveIK_DLS(model, data, l_EE, target_pose_L, q_ik);
            dualarm.SolveIK_DLS(model, data, r_EE, target_pose_R, q_ik);

            for (int i = 0; i < DoF; i++) {
                dual_arm_targetp[i] = q_ik(i);
                dual_arm_targetv[i] = 0.0; 
                dual_arm_targeta_vec(i) = 0.0;
            }

            // 💡 핵심 덮어쓰기: IK 연산이 끝난 후, 오직 머리(10번 관절)만 수동으로 조작!
            if (do_aruco_pick) {
                if (pick_step == 2) {
                    // 1초 동안 서서히 고개를 숙이는 애니메이션
                    double target_angle = -40.0 * M_PI / 180.0;
                    double progress = (double)traj_cnt / cartesian_p_trajectory_L.rows();
                    dual_arm_targetp[10] = head_start_angle + (target_angle - head_start_angle) * progress;
                } else {
                    // 이후에는 계속 40도로 고정
                    dual_arm_targetp[10] = -40.0 * M_PI / 180.0;
                }
            }

            dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
            for (int i = 0; i < DoF; i++){
                dual_arm_targeta_vec(i) += PD_acc[i];
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