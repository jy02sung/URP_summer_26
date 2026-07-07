#include "dual_arm_function.cpp"
#include <tf/transform_listener.h>
#include <vector>

// ArUco 인식 결과 (aruco_ros/single) - 카메라 광학 프레임 기준 pose
geometry_msgs::PoseStamped aruco_pose_cam;
bool aruco_pose_received = false;

void msgCallbackArucoPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_pose_cam = *msg;
    aruco_pose_received = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
// /dual_arm/joint_states 의 name 배열은 조인트 이름 알파벳순으로 정렬되어 온다(실측 확인).
// Head_pitch_joint(0) < Head_yaw_joint(1) < L_elbow_joint(2) < L_shoulder_pitch_joint(3) <
// L_shoulder_roll_joint(4) < L_shoulder_yaw_joint(5) < R_elbow_joint(6) < R_shoulder_pitch_joint(7) <
// R_shoulder_roll_joint(8) < R_shoulder_yaw_joint(9) < Waist_joint(10)
void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        waist_jointp[0] = msg->position[10];
        waist_jointv[0] = msg->velocity[10];
        waist_torque[0] = msg->effort[10];
    }

void msgCallbackHeadJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        head_jointp[0] = msg->position[1];   // yaw
        head_jointv[0] = msg->velocity[1];
        head_torque[0] = msg->effort[1];

        head_jointp[1] = msg->position[0];   // pitch
        head_jointv[1] = msg->velocity[0];
        head_torque[1] = msg->effort[0];
    }

void msgCallbackLeftArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        left_arm_jointp[0] = msg->position[3];
        left_arm_jointv[0] = msg->velocity[3];
        left_arm_torque[0] = msg->effort[3];

        left_arm_jointp[1] = msg->position[4];
        left_arm_jointv[1] = msg->velocity[4];
        left_arm_torque[1] = msg->effort[4];

        left_arm_jointp[2] = msg->position[5];
        left_arm_jointv[2] = msg->velocity[5];
        left_arm_torque[2] = msg->effort[5];

        left_arm_jointp[3] = msg->position[2];
        left_arm_jointv[3] = msg->velocity[2];
        left_arm_torque[3] = msg->effort[2];
    }

void msgCallbackRightArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        right_arm_jointp[0] = msg->position[7];
        right_arm_jointv[0] = msg->velocity[7];
        right_arm_torque[0] = msg->effort[7];

        right_arm_jointp[1] = msg->position[8];
        right_arm_jointv[1] = msg->velocity[8];
        right_arm_torque[1] = msg->effort[8];

        right_arm_jointp[2] = msg->position[9];
        right_arm_jointv[2] = msg->velocity[9];
        right_arm_torque[2] = msg->effort[9];

        right_arm_jointp[3] = msg->position[6];
        right_arm_jointv[3] = msg->velocity[6];
        right_arm_torque[3] = msg->effort[6];
    }

// F/T 센서 콜백 (임피던스 제어의 F_ext로 사용)
void msgCallbackLeftFTSensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    left_ft_force(0) = msg->wrench.force.x;
    left_ft_force(1) = msg->wrench.force.y;
    left_ft_force(2) = msg->wrench.force.z;
}

void msgCallbackRightFTSensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    right_ft_force(0) = msg->wrench.force.x;
    right_ft_force(1) = msg->wrench.force.y;
    right_ft_force(2) = msg->wrench.force.z;
}

// 수동 오버라이드용 (idle 상태이거나 mode 0/1/2 테스트 시에만 유효.
// vision pick(mode 3) 재생 중에는 main.cpp가 세그먼트 기반으로 매 스텝 덮어씀)
void msgCallbackTaskPhase(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data >= PHASE_APPROACH && msg->data <= PHASE_RETURN) {
        task_phase = msg->data;
    }
}

void msgCallbackDualArmCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    command_mode = (int)msg->data[0];   // data[0] = 0/1/2/3

    if (command_mode == 0) {
        // modeling: data[1..DoF] = 관절각 DoF개
        for (int i = 0; i < DoF; i++) {
            dual_arm_commandp[i] = msg->data[i + 1];
        }
    }
    else if (command_mode == 3) {
        // vision pick: data[1..3] = 이송(transport) 목표 위치 (world 기준)
        for (int i = 0; i < 3; i++) {
            dual_arm_commandx[i] = msg->data[i + 1];
        }
    }
    else {
        // joint(1) / cartesian(2): data[1..6] = 목표 EE 위치
        for (int i = 0; i < 6; i++) {
            dual_arm_commandx[i] = msg->data[i + 1];
        }
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
    ros::Subscriber sub_waist_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackWaistArmJointState);
    ros::Subscriber sub_head_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackHeadJointState);
    ros::Subscriber sub_left_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackLeftArmJointState);
    ros::Subscriber sub_right_arm_joint_angle = nh.subscribe("/dual_arm/joint_states", 100, msgCallbackRightArmJointState);
    ros::Subscriber sub_dual_arm_cmd = nh.subscribe("/dual_arm/DualArmCmd_sim", 100, msgCallbackDualArmCmd);
    ros::Subscriber sub_aruco_pose = nh.subscribe("/aruco_ros/pose", 10, msgCallbackArucoPose);
    ros::Subscriber sub_left_ft_sensor = nh.subscribe("/dual_arm/left_ft_sensor", 100, msgCallbackLeftFTSensor);
    ros::Subscriber sub_right_ft_sensor = nh.subscribe("/dual_arm/right_ft_sensor", 100, msgCallbackRightFTSensor);
    ros::Subscriber sub_task_phase = nh.subscribe("/dual_arm/TaskPhase", 10, msgCallbackTaskPhase);

    // TaskPhase 자동전환 알림 + 궤적 실행 완료 알림 (vision pick 진행상황을 외부에서 관측 가능)
    ros::Publisher task_phase_pub = nh.advertise<std_msgs::Int32>("/dual_arm/TaskPhase", 10);
    ros::Publisher dual_armtraj_done_pub = nh.advertise<std_msgs::Bool>("/dual_arm/TrajectoryDone", 10);
    int prev_task_phase = task_phase;   // 값이 바뀔 때만 발행 (edge-trigger)

    // ArUco pose(카메라 프레임) -> world 프레임 변환용
    tf::TransformListener tfListener;

    ros::Rate loop_rate(1000);
    ros::spinOnce();
    
    // 조인트 명령 메시지 객체 선언
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 head_yaw_joint_msg, head_pitch_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg;

    // 고정된 URDF 파일 경로 사용
    string urdf_filename = "/home/jungmin/friend_ws/src/Dual_Arm_ROS_Simulation/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // ===== 추가: EE 프레임 ID는 불변이므로 루프 밖에서 한 번만 구함 =====
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    while(ros::ok())
    {

        dual_arm_jointp[0] = waist_jointp[0];
        dual_arm_jointp[1] = head_jointp[0];
        dual_arm_jointp[2] = head_jointp[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp[i + 3] = left_arm_jointp[i];
            dual_arm_jointp[i + 7] = right_arm_jointp[i];
        }

        dual_arm_jointv[0] = waist_jointv[0];
        dual_arm_jointv[1] = head_jointv[0];
        dual_arm_jointv[2] = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv[i + 3] = left_arm_jointv[i];
            dual_arm_jointv[i + 7] = right_arm_jointv[i];
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
        dual_arm_jointp_vec(1) = head_jointp[0];
        dual_arm_jointp_vec(2) = head_jointp[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointp_vec(i + 3) = left_arm_jointp[i];
            dual_arm_jointp_vec(i + 7) = right_arm_jointp[i];
        }

        dual_arm_jointv_vec(0) = waist_jointv[0];
        dual_arm_jointv_vec(1) = head_jointv[0];
        dual_arm_jointv_vec(2) = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_vec(i + 3) = left_arm_jointv[i];
            dual_arm_jointv_vec(i + 7) = right_arm_jointv[i];
        }

        dual_arm_jointv_lpf_vec(0) = waist_jointv[0];
        dual_arm_jointv_lpf_vec(1) = head_jointv[0];
        dual_arm_jointv_lpf_vec(2) = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_lpf_vec(i + 3) = left_arm_jointv[i];
            dual_arm_jointv_lpf_vec(i + 7) = right_arm_jointv[i];
        }



        if(callback == true){
            bool new_trajectory_built = true;  // command_mode==3이 실패하면 false로 바뀌어 기존 궤적 재생을 유지
            // 현재 관절각을 initp(궤적 시작점) 및 IK 시드로 저장
            dual_arm_initp[0] = waist_jointp[0];
            dual_arm_initp[1] = head_jointp[0];
            dual_arm_initp[2] = head_jointp[1];
            for (int i = 0; i < 4; i++) {
                dual_arm_initp[i + 3] = left_arm_jointp[i];
                dual_arm_initp[i + 7] = right_arm_jointp[i];
            }
            for (int i = 0; i < DoF; i++) q_ik_seed(i) = dual_arm_initp[i];

            // ===== 모드 0: modeling (기존 방식, 관절각 직접) =====
            if (command_mode == 0) {
                dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp,
                    dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
                // 수동 modeling 명령은 임피던스 대상이 아님 -> 전 구간 접근 단계로 태깅
                dual_arm_phase_trajectory.setConstant(dual_arm_jointp_trajectory.rows(), PHASE_APPROACH);
            }

            // ===== 모드 1: joint sim (위치 IK 1회 → 관절공간 5차 궤적) =====
            else if (command_mode == 1) {
                Vector3d tL(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);
                Vector3d tR(dual_arm_commandx[3], dual_arm_commandx[4], dual_arm_commandx[5]);

                dualarm.SolveIK_Position(model, data, l_EE, r_EE, tL, tR, q_ik_seed, q_ik_result);

                for (int i = 0; i < DoF; i++) dual_arm_commandp[i] = q_ik_result(i);

                dualarm.JointTrajectoryQuintic(dual_arm_initp, dual_arm_commandp,
                    dual_arm_jointp_trajectory, dual_arm_jointv_trajectory, dual_arm_jointa_trajectory);
                // 수동 joint sim 명령은 임피던스 대상이 아님 -> 전 구간 접근 단계로 태깅
                dual_arm_phase_trajectory.setConstant(dual_arm_jointp_trajectory.rows(), PHASE_APPROACH);
            }

            // ===== 모드 2: cartesian sim (직교 직선 → 매 스텝 IK → 관절각 궤적) =====
            else if (command_mode == 2) {
                for (int i = 0; i < DoF; i++) {
                    dual_arm_jointp_vec(i) = dual_arm_initp[i];
                }
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

                    for (int i = 0; i < DoF; i++) {
                        dual_arm_jointp_trajectory(k,i) = q_k(i);
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

                // 수동 cartesian sim 명령은 임피던스 대상이 아님 -> 전 구간 접근 단계로 태깅
                dual_arm_phase_trajectory.setConstant(steps, PHASE_APPROACH);
            }

            // ===== 모드 3: vision pick (ArUco 검출 -> world 변환 -> 접근/파지/이송) =====
            else if (command_mode == 3) {
                if (!aruco_pose_received) {
                    ROS_WARN("No /aruco_ros/pose received yet - vision pick skipped.");
                    new_trajectory_built = false;
                }
                else {
                    // 1) 카메라 프레임 pose를 world 프레임으로 변환 (ros::Time(0) = 최신 가용 tf)
                    geometry_msgs::PoseStamped pose_in = aruco_pose_cam;
                    pose_in.header.stamp = ros::Time(0);
                    geometry_msgs::PoseStamped object_world;
                    bool tf_ok = true;
                    try {
                        tfListener.transformPose("world", pose_in, object_world);
                    }
                    catch (tf::TransformException& ex) {
                        ROS_ERROR("Vision pick TF transform failed: %s", ex.what());
                        tf_ok = false;
                        new_trajectory_built = false;
                    }

                    if (tf_ok) {
                        Vector3d obj(object_world.pose.position.x,
                                     object_world.pose.position.y,
                                     object_world.pose.position.z);
                        Vector3d transport_pt(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);

                        // 양팔 동시 파지 오프셋(물체를 y축 양쪽에서 감싸는 형태)
                        const double straddle_offset = 0.05; // 1) 접근 시 물체 양옆 간격
                        const double grasp_offset    = 0.02; // 2) 파지 시 좁힌 간격 (물체에 밀착)
                        const double lift_offset     = 0.10; // 들어올리는 높이

                        // 현재 양팔 EE 위치 (시작점 + 마지막 6) 복귀 목표)
                        pinocchio::forwardKinematics(model, data, q_ik_seed);
                        pinocchio::updateFramePlacements(model, data);
                        Vector3d start_L = data.oMf[l_EE].translation();
                        Vector3d start_R = data.oMf[r_EE].translation();

                        std::vector<MatrixXd> pos_segs, vel_segs, acc_segs;
                        std::vector<int> seg_phase;   // 세그먼트별 TaskPhase 태그 (재생 중 자동 전환용)
                        VectorXd seed_vec = q_ik_seed;
                        double prev_q[DoF];
                        for (int i = 0; i < DoF; i++) prev_q[i] = dual_arm_initp[i];

                        auto addSegment = [&](const Vector3d& targetL, const Vector3d& targetR) {
                            VectorXd q_result;
                            dualarm.SolveIK_Position(model, data, l_EE, r_EE, targetL, targetR, seed_vec, q_result);
                            double q_cmd[DoF];
                            for (int i = 0; i < DoF; i++) q_cmd[i] = q_result(i);
                            MatrixXd p, v, a;
                            dualarm.JointTrajectoryQuintic(prev_q, q_cmd, p, v, a);
                            pos_segs.push_back(p);
                            vel_segs.push_back(v);
                            acc_segs.push_back(a);
                            for (int i = 0; i < DoF; i++) prev_q[i] = q_cmd[i];
                            seed_vec = q_result;
                        };

                        // 1) 양팔 동시 접근: 물체 양옆(y ±straddle_offset)으로 -> 접근 단계(임피던스 OFF)
                        addSegment(obj + Vector3d(0,  straddle_offset, 0),
                                   obj + Vector3d(0, -straddle_offset, 0));
                        seg_phase.push_back(PHASE_APPROACH);

                        // 2) 양팔 동시 파지: 간격을 좁혀 물체에 밀착 -> 파지 시작, 임피던스 ON
                        addSegment(obj + Vector3d(0,  grasp_offset, 0),
                                   obj + Vector3d(0, -grasp_offset, 0));
                        seg_phase.push_back(PHASE_GRASP_TO_PLACE);

                        // 3) 양팔 동시 들어올리기 (간격 유지한 채 위로) -> 임피던스 ON 유지
                        addSegment(obj + Vector3d(0,  grasp_offset, lift_offset),
                                   obj + Vector3d(0, -grasp_offset, lift_offset));
                        seg_phase.push_back(PHASE_GRASP_TO_PLACE);

                        // 4) 양팔 동시 이송: 목표 좌표 위 lift_offset 높이로 이동 -> 임피던스 ON 유지
                        addSegment(transport_pt + Vector3d(0,  grasp_offset, lift_offset),
                                   transport_pt + Vector3d(0, -grasp_offset, lift_offset));
                        seg_phase.push_back(PHASE_GRASP_TO_PLACE);

                        // 5) 양팔 동시 내려놓기 -> 내려놓기 완료 시점까지 임피던스 ON
                        addSegment(transport_pt + Vector3d(0,  grasp_offset, 0),
                                   transport_pt + Vector3d(0, -grasp_offset, 0));
                        seg_phase.push_back(PHASE_GRASP_TO_PLACE);

                        // 6) 양팔 동시 원래 위치로 복귀 -> 복귀 단계, 임피던스 OFF
                        addSegment(start_L, start_R);
                        seg_phase.push_back(PHASE_RETURN);

                        ROS_INFO("Vision pick(dual-arm): object(world)=[%.3f %.3f %.3f], transport=[%.3f %.3f %.3f]",
                                 obj.x(), obj.y(), obj.z(),
                                 transport_pt.x(), transport_pt.y(), transport_pt.z());

                        int total_rows = 0;
                        for (auto& s : pos_segs) total_rows += s.rows();
                        dual_arm_jointp_trajectory.resize(total_rows, DoF);
                        dual_arm_jointv_trajectory.resize(total_rows, DoF);
                        dual_arm_jointa_trajectory.resize(total_rows, DoF);
                        dual_arm_phase_trajectory.resize(total_rows);
                        int offset = 0;
                        for (size_t k = 0; k < pos_segs.size(); ++k) {
                            int r = pos_segs[k].rows();
                            dual_arm_jointp_trajectory.block(offset, 0, r, DoF) = pos_segs[k];
                            dual_arm_jointv_trajectory.block(offset, 0, r, DoF) = vel_segs[k];
                            dual_arm_jointa_trajectory.block(offset, 0, r, DoF) = acc_segs[k];
                            dual_arm_phase_trajectory.segment(offset, r).setConstant(seg_phase[k]);
                            offset += r;
                        }
                    }
                }
            }

            if (new_trajectory_built) {
                traj_cnt = 0;
                traj_done_published = false;   // 새 궤적 시작 -> 완료 알림 다시 대기
            }
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

            // vision pick(mode 3) 재생 중이면 현재 행에 태깅된 phase로 자동 전환.
            // (mode 0/1/2는 전 구간 PHASE_APPROACH로 태깅되어 있어 임피던스가 자동으로 켜지지 않음)
            if (traj_cnt < dual_arm_phase_trajectory.size()) {
                task_phase = dual_arm_phase_trajectory(traj_cnt);
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

            // 궤적(복귀 포함) 실행이 막 끝난 시점: 접근 단계로 리셋 + 완료 알림, 딱 한 번만
            if (!traj_done_published) {
                task_phase = PHASE_APPROACH;   // 복귀 완료 -> 접근 단계로 리셋 (임피던스 OFF)

                std_msgs::Bool traj_done_msg;
                traj_done_msg.data = true;
                dual_armtraj_done_pub.publish(traj_done_msg);
                traj_done_published = true;
            }
        }

        // TaskPhase 값이 바뀐 순간에만 발행 (외부에서 전환 시점을 관측 가능)
        if (task_phase != prev_task_phase) {
            std_msgs::Int32 phase_msg;
            phase_msg.data = task_phase;
            task_phase_pub.publish(phase_msg);
            prev_task_phase = task_phase;
        }

        for (int i = 0; i < DoF; ++i) {
            dual_arm_targetp_vec(i) = dual_arm_targetp[i];
        }

        // ===== 임피던스 제어 (파지~내려놓기 구간에서만 활성화, 그 외에는 기존 PD+RNEA만) =====
        // Md*e_ddot + Bd*e_dot + Kd*e = F_ext  (e = 실제 EE 위치 - 목표 EE 위치, world frame, 위치 3축만)
        // 여기서 얻은 e_ddot(가상 응답 가속도)를 댐핑 의사역행렬로 관절가속도로 변환해
        // dual_arm_targeta_vec 에 더해준다. -> RNEA가 M(q)*a 항을 통해 자동으로 추가 토크로 반영.
        bool impedance_active = (task_phase == PHASE_GRASP_TO_PLACE);
        if (impedance_active) {
            // 실제 관절 상태에서의 FK/자코비안
            pinocchio::computeJointJacobians(model, data, dual_arm_jointp_vec);
            pinocchio::updateFramePlacements(model, data);

            Vector3d xL_actual = data.oMf[l_EE].translation();
            Vector3d xR_actual = data.oMf[r_EE].translation();
            Matrix3d RL_actual = data.oMf[l_EE].rotation();
            Matrix3d RR_actual = data.oMf[r_EE].rotation();

            pinocchio::Data::Matrix6x JL_full(6, model.nv); JL_full.setZero();
            pinocchio::Data::Matrix6x JR_full(6, model.nv); JR_full.setZero();
            pinocchio::getFrameJacobian(model, data, l_EE, pinocchio::LOCAL_WORLD_ALIGNED, JL_full);
            pinocchio::getFrameJacobian(model, data, r_EE, pinocchio::LOCAL_WORLD_ALIGNED, JR_full);
            MatrixXd JL = JL_full.topRows<3>();   // 위치 3행만 (DoF 열)
            MatrixXd JR = JR_full.topRows<3>();

            Vector3d xL_dot_actual = JL * dual_arm_jointv_vec;
            Vector3d xR_dot_actual = JR * dual_arm_jointv_vec;

            // 목표(지령) EE 위치/속도 (target 궤적 기준 FK, 같은 자코비안으로 근사)
            pinocchio::forwardKinematics(model, data, dual_arm_targetp_vec);
            pinocchio::updateFramePlacements(model, data);
            Vector3d xL_d = data.oMf[l_EE].translation();
            Vector3d xR_d = data.oMf[r_EE].translation();

            VectorXd targetv_vec(DoF);
            for (int i = 0; i < DoF; i++) targetv_vec(i) = dual_arm_targetv[i];
            Vector3d xL_d_dot = JL * targetv_vec;
            Vector3d xR_d_dot = JR * targetv_vec;

            Vector3d eL = xL_actual - xL_d;
            Vector3d eR = xR_actual - xR_d;
            Vector3d eL_dot = xL_dot_actual - xL_d_dot;
            Vector3d eR_dot = xR_dot_actual - xR_d_dot;

            // F/T 센서 힘: 센서가 EE 프레임과 동일 방향으로 장착되었다고 가정하고 world frame으로 변환
            Vector3d F_ext_L = RL_actual * left_ft_force;
            Vector3d F_ext_R = RR_actual * right_ft_force;

            Vector3d eL_ddot, eR_ddot;
            for (int k = 0; k < 3; k++) {
                eL_ddot(k) = (F_ext_L(k) - Bd_left[k]*eL_dot(k) - Kd_imp_left[k]*eL(k)) / Md_left[k];
                eR_ddot(k) = (F_ext_R(k) - Bd_right[k]*eR_dot(k) - Kd_imp_right[k]*eR(k)) / Md_right[k];
            }

            VectorXd dq_ddot_L = dualarm.DampedPinv(JL, IMPEDANCE_DLS_LAMBDA) * eL_ddot;
            VectorXd dq_ddot_R = dualarm.DampedPinv(JR, IMPEDANCE_DLS_LAMBDA) * eR_ddot;

            dual_arm_targeta_vec += dq_ddot_L + dq_ddot_R;
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

        // Position Control
        #if ARMCTRLMODE == POSITION
            waist_joint_msg.data            = dual_arm_targetp[0];
            head_yaw_joint_msg.data         = dual_arm_targetp[1];
            head_pitch_joint_msg.data       = dual_arm_targetp[2];
            shoulder_pitch_l_joint_msg.data = dual_arm_targetp[3];
            shoulder_roll_l_joint_msg.data  = dual_arm_targetp[4];
            shoulder_yaw_l_joint_msg.data   = dual_arm_targetp[5];
            elbow_l_joint_msg.data          = dual_arm_targetp[6];
            shoulder_pitch_r_joint_msg.data = dual_arm_targetp[7];
            shoulder_roll_r_joint_msg.data  = dual_arm_targetp[8];
            shoulder_yaw_r_joint_msg.data   = dual_arm_targetp[9];
            elbow_r_joint_msg.data          = dual_arm_targetp[10];

        // Effort Control
        #elif ARMCTRLMODE == EFFORT
            waist_joint_msg.data            = target_torque[0];
            head_yaw_joint_msg.data         = target_torque[1];
            head_pitch_joint_msg.data       = target_torque[2];
            shoulder_pitch_l_joint_msg.data = target_torque[3];
            shoulder_roll_l_joint_msg.data  = target_torque[4];
            shoulder_yaw_l_joint_msg.data   = target_torque[5];
            elbow_l_joint_msg.data          = target_torque[6];
            shoulder_pitch_r_joint_msg.data = target_torque[7];
            shoulder_roll_r_joint_msg.data  = target_torque[8];
            shoulder_yaw_r_joint_msg.data   = target_torque[9];
            elbow_r_joint_msg.data          = target_torque[10];

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