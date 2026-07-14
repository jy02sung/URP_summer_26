#include "dual_arm_function.cpp"
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <vector>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelState.h>

// place 목표 지점(이송 목표)을 눈으로 확인할 수 있도록 vision pick 명령이 들어올 때마다 스폰하는
// 표시용 모델. static이라 물리엔진과 무관 - pedestal은 물체가 도착했을 때 실제로 받쳐주기도 하고,
// 위의 보라색 구는 명령으로 입력한 좌표(dual_arm_commandx) 그 자체를 정확히 표시한다.
// 매번 같은 이름("place_indicator")으로 스폰하므로, 재사용 전에 항상 delete부터 해서 이전 실행의
// 잔여물이 안 남게 한다.
const string PLACE_INDICATOR_SDF = R"(
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="place_indicator">
    <static>true</static>
    <link name="link">
      <collision name="pedestal_collision">
        <pose>0 0 -0.10 0 0 0</pose>
        <geometry><box><size>0.12 0.12 0.10</size></box></geometry>
      </collision>
      <visual name="pedestal_visual">
        <pose>0 0 -0.10 0 0 0</pose>
        <geometry><box><size>0.12 0.12 0.10</size></box></geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.2 0.8 0.2 1</diffuse>
        </material>
      </visual>
      <visual name="target_point_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
        <material>
          <ambient>1 0 1 1</ambient>
          <diffuse>1 0 1 1</diffuse>
          <emissive>0.6 0 0.6 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
)";

// ArUco 인식 결과 (aruco_ros/single) - 카메라 광학 프레임 기준 pose
geometry_msgs::PoseStamped aruco_pose_cam;
bool aruco_pose_received = false;
unsigned long aruco_pose_seq = 0;   // 콜백마다 증가. Head 스캔 중 "새 프레임 도착"을 감지하기 위한 시퀀스 번호
                                     // (aruco_pose_cam 값 자체는 새 메시지가 올 때까지 안 바뀌므로 값만 봐서는 새 프레임인지 알 수 없음)

bool waist_state_received = false;
bool head_state_received = false;
bool left_arm_state_received = false;
bool right_arm_state_received = false;

void msgCallbackArucoPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_pose_cam = *msg;
    aruco_pose_received = true;
    aruco_pose_seq++;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Head 마커 탐색 (PHASE_SCAN)
// command_mode==3(vision pick) 진입 시 Head를 고정 자세(HEAD_SCAN_YAW/PITCH)로 이동시키고 ArUco 검출을 기다린다.
// P0 진단 결과: Head_pitch는 +방향이 아래(물체 쪽)를 향한다 (좌표계 직관과 반대).
// 정지 후 0.5s 대기 -> 그 이후 도착하는 /aruco_ros/pose를 연속 3프레임 비교해서 서로 1cm 이내로
// 일치하면 검출 확정. 확인 시간이 타임아웃을 넘으면 ROS_WARN을 띄우고 실패 처리한다.
enum ScanStep { SCAN_MOVE, SCAN_SETTLE, SCAN_CHECK };

bool scan_active = false;
int scan_step = SCAN_MOVE;
int scan_wait_cnt = 0;
unsigned long scan_last_seq = 0;
std::vector<Vector3d> scan_match_buf;  // 연속 프레임 일치 판정용 버퍼 (카메라 프레임 좌표)
double scan_q[DoF] = {0,};             // 스캔 진행 중 "현재 명령 관절각" (head만 갱신, 나머지는 스캔 시작 시점 값 유지)

const double HEAD_SCAN_YAW            = 0.0;
const double HEAD_SCAN_PITCH_CANDIDATES[] = {
    0.5236,  // 30 deg
    0.7854,  // 45 deg
    1.0472   // 60 deg
};
const int    HEAD_SCAN_PITCH_COUNT    = sizeof(HEAD_SCAN_PITCH_CANDIDATES) / sizeof(HEAD_SCAN_PITCH_CANDIDATES[0]);
const int    SCAN_SETTLE_TICKS        = 500;     // 0.5s @ 1000Hz - 정지 후 카메라/인식 안정화 대기
const int    SCAN_CHECK_TIMEOUT_TICKS = 4000;    // 4s - 이 안에 3프레임 일치를 못 찾으면 실패 처리
                                                  // (실측: aruco_ros 인식 속도가 ~1.5~2Hz에 간헐적으로 최대 ~1s 갭이 있음)
const int    SCAN_MATCH_FRAMES        = 3;       // 연속 일치 판정에 필요한 프레임 수
const double SCAN_MATCH_TOL           = 0.01;    // [m] 연속 프레임 간 허용 오차
int scan_pitch_index = 0;

const double STARTUP_WAIST = 0.0;
const double STARTUP_HEAD_YAW = 0.0;
const double STARTUP_HEAD_PITCH = 0.0;
const double STARTUP_L_SHOULDER_PITCH = 0.68;
const double STARTUP_L_SHOULDER_ROLL = 0.0;
const double STARTUP_L_SHOULDER_YAW = -0.20;
const double STARTUP_L_ELBOW = -0.45;
const double STARTUP_R_SHOULDER_PITCH = 0.68;
const double STARTUP_R_SHOULDER_ROLL = 0.0;
const double STARTUP_R_SHOULDER_YAW = 0.20;
const double STARTUP_R_ELBOW = -0.45;
const int STARTUP_HOLD_TICKS = 1000;           // 1.0s @ 1000Hz
const double STARTUP_MOVE_DURATION = 3.0;      // [s] 초기 자세로 천천히 이동
const double STARTUP_SETTLE_VEL_NORM = 0.35;   // [rad/s]

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
// /dual_arm/joint_states 의 name 배열은 조인트 이름 알파벳순으로 정렬되어 온다(실측 확인).
// Head_pitch_joint(0) < Head_yaw_joint(1) < L_elbow_joint(2) < L_shoulder_pitch_joint(3) <
// L_shoulder_roll_joint(4) < L_shoulder_yaw_joint(5) < R_elbow_joint(6) < R_shoulder_pitch_joint(7) <
// R_shoulder_roll_joint(8) < R_shoulder_yaw_joint(9) < Waist_joint(10)
void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        waist_state_received = true;
        waist_jointp[0] = msg->position[10];
        waist_jointv[0] = msg->velocity[10];
        waist_torque[0] = msg->effort[10];
    }

void msgCallbackHeadJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        head_state_received = true;
        head_jointp[0] = msg->position[1];   // yaw
        head_jointv[0] = msg->velocity[1];
        head_torque[0] = msg->effort[1];

        head_jointp[1] = msg->position[0];   // pitch
        head_jointv[1] = msg->velocity[0];
        head_torque[1] = msg->effort[0];
    }

void msgCallbackLeftArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        left_arm_state_received = true;
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
        right_arm_state_received = true;
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

// F/T 센서 콜백 (어드미턴스 제어의 F_ext로 사용)
void msgCallbackLeftFTSensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    left_ft_force(0) = msg->wrench.force.x;
    left_ft_force(1) = msg->wrench.force.y;
    left_ft_force(2) = msg->wrench.force.z;
    left_ft_torque(0) = msg->wrench.torque.x;
    left_ft_torque(1) = msg->wrench.torque.y;
    left_ft_torque(2) = msg->wrench.torque.z;
}

void msgCallbackRightFTSensor(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    right_ft_force(0) = msg->wrench.force.x;
    right_ft_force(1) = msg->wrench.force.y;
    right_ft_force(2) = msg->wrench.force.z;
    right_ft_torque(0) = msg->wrench.torque.x;
    right_ft_torque(1) = msg->wrench.torque.y;
    right_ft_torque(2) = msg->wrench.torque.z;
}

// 수동 오버라이드용 (idle 상태이거나 mode 0/1/2 테스트 시에만 유효.
// vision pick(mode 3) 재생 중에는 main.cpp가 세그먼트 기반으로 매 스텝 덮어씀)
void msgCallbackTaskPhase(const std_msgs::Int32::ConstPtr& msg)
{
    if (!scan_active && command_mode != 3 &&
        msg->data >= PHASE_APPROACH && msg->data <= PHASE_RETURN) {
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
    ROS_INFO("DualArmCmd callback: mode=%d", command_mode);
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

    // place 목표 지점 표시/받침대용 스폰-삭제 서비스 클라이언트
    ros::ServiceClient spawn_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient delete_model_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    // vision pick 명령이 들어올 때마다 이송 목표(target) 위치에 place_indicator를 새로 스폰.
    // 이전 실행에서 남아있을 수 있으므로 스폰 전에 항상 delete부터 시도한다(없으면 실패해도 무해).
    auto spawnPlaceIndicator = [&](const Vector3d& target) {
        gazebo_msgs::DeleteModel del_srv;
        del_srv.request.model_name = "place_indicator";
        delete_model_client.call(del_srv);

        gazebo_msgs::SpawnModel spawn_srv;
        spawn_srv.request.model_name = "place_indicator";
        spawn_srv.request.model_xml = PLACE_INDICATOR_SDF;
        spawn_srv.request.reference_frame = "world";
        spawn_srv.request.initial_pose.position.x = target.x();
        spawn_srv.request.initial_pose.position.y = target.y();
        spawn_srv.request.initial_pose.position.z = target.z();
        spawn_srv.request.initial_pose.orientation.w = 1.0;
        spawn_model_client.call(spawn_srv);
    };

    ros::Rate loop_rate(1000);
    ros::spinOnce();
    
    // 조인트 명령 메시지 객체 선언
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 head_yaw_joint_msg, head_pitch_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg;

    // 고정된 URDF 파일 경로 사용
    string urdf_filename = ros::package::getPath("dual_arm") + "/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // ===== 추가: EE 프레임 ID는 불변이므로 루프 밖에서 한 번만 구함 =====
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");

    dual_arm_jointp_trajectory.resize(1, DoF);
    dual_arm_jointv_trajectory.resize(1, DoF);
    dual_arm_jointa_trajectory.resize(1, DoF);
    dual_arm_jointv_trajectory.setZero();
    dual_arm_jointa_trajectory.setZero();
    dual_arm_jointp_trajectory <<
        STARTUP_WAIST,
        STARTUP_HEAD_YAW,
        STARTUP_HEAD_PITCH,
        STARTUP_L_SHOULDER_PITCH,
        STARTUP_L_SHOULDER_ROLL,
        STARTUP_L_SHOULDER_YAW,
        STARTUP_L_ELBOW,
        STARTUP_R_SHOULDER_PITCH,
        STARTUP_R_SHOULDER_ROLL,
        STARTUP_R_SHOULDER_YAW,
        STARTUP_R_ELBOW;

    enum StartupState { STARTUP_WAIT_FOR_STATE, STARTUP_HOLD_CURRENT, STARTUP_MOVE_TO_TARGET, STARTUP_COMPLETE };
    StartupState startup_state = STARTUP_WAIT_FOR_STATE;
    int startup_hold_cnt = 0;
    double startup_hold_q[DoF] = {0.0,};
    double startup_target_q[DoF] = {
        STARTUP_WAIST,
        STARTUP_HEAD_YAW,
        STARTUP_HEAD_PITCH,
        STARTUP_L_SHOULDER_PITCH,
        STARTUP_L_SHOULDER_ROLL,
        STARTUP_L_SHOULDER_YAW,
        STARTUP_L_ELBOW,
        STARTUP_R_SHOULDER_PITCH,
        STARTUP_R_SHOULDER_ROLL,
        STARTUP_R_SHOULDER_YAW,
        STARTUP_R_ELBOW
    };

    // ===== Head 자동 스캔: 다음 웨이포인트로 Head만 이동시키는 단일 세그먼트를 만들어 재생 준비 =====
    // 다른 관절(waist/양팔)은 scan_q에 저장된 직전 값을 그대로 유지한다.
    auto startScanMoveTo = [&](double yaw, double pitch) {
        double q_cmd[DoF];
        for (int i = 0; i < DoF; i++) q_cmd[i] = scan_q[i];
        q_cmd[1] = yaw;
        q_cmd[2] = pitch;

        MatrixXd p, v, a;
        dualarm.JointTrajectoryQuintic(scan_q, q_cmd, p, v, a);
        dual_arm_jointp_trajectory = p;
        dual_arm_jointv_trajectory = v;
        dual_arm_jointa_trajectory = a;
        dual_arm_phase_trajectory.resize(p.rows());
        dual_arm_phase_trajectory.setConstant(PHASE_SCAN);

        for (int i = 0; i < DoF; i++) scan_q[i] = q_cmd[i];
        traj_cnt = 0;
        traj_done_published = false;
        scan_step = SCAN_MOVE;
    };

    auto advanceScanPose = [&]() {
        scan_pitch_index++;
        if (scan_pitch_index >= HEAD_SCAN_PITCH_COUNT) {
            return false;
        }

        const double next_pitch = HEAD_SCAN_PITCH_CANDIDATES[scan_pitch_index];
        ROS_INFO("Head scan: retrying with deeper pitch %.1fdeg.", next_pitch * rad2deg);
        startScanMoveTo(HEAD_SCAN_YAW, next_pitch);
        return true;
    };

    auto buildFixedDurationJointTrajectory = [&](const double* q_ini, const double* q_cmd, double duration_sec) {
        int step = std::max(2, static_cast<int>(std::round(duration_sec / SAMPLING_TIME_TRAJ)));
        dual_arm_jointp_trajectory.resize(step, DoF);
        dual_arm_jointv_trajectory.resize(step, DoF);
        dual_arm_jointa_trajectory.resize(step, DoF);
        dual_arm_phase_trajectory.resize(step);
        dual_arm_phase_trajectory.setConstant(PHASE_APPROACH);

        for (int i = 0; i < DoF; ++i) {
            const double q0 = q_ini[i];
            const double qf = q_cmd[i];
            const double dq = qf - q0;

            for (int j = 0; j < step; ++j) {
                const double s = (step == 1) ? 1.0 : static_cast<double>(j) / static_cast<double>(step - 1);
                const double s2 = s * s;
                const double s3 = s2 * s;
                const double s4 = s3 * s;
                const double s5 = s4 * s;

                const double blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
                const double dblend_ds = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
                const double d2blend_ds2 = 60.0 * s - 180.0 * s2 + 120.0 * s3;

                dual_arm_jointp_trajectory(j, i) = q0 + dq * blend;
                dual_arm_jointv_trajectory(j, i) = dq * dblend_ds / duration_sec;
                dual_arm_jointa_trajectory(j, i) = dq * d2blend_ds2 / (duration_sec * duration_sec);
            }
        }
    };

    // ===== Head 스캔으로 확정된 검출 결과로 기존 양팔 파지 파이프라인(접근~복귀 6세그먼트)을 생성 =====
    // base_q: 파이프라인 시작 시점의 "현재 관절각"(waist/양팔은 스캔 시작 시점 값, head는 스캔이 멈춘 실제 웨이포인트).
    // 반환값 false면 TF 변환 실패 -> 궤적 생성 안 됨(호출부에서 실패 처리).
    auto buildGraspPipelineFromDetection = [&](const double* base_q) -> bool {
        geometry_msgs::PoseStamped pose_in = aruco_pose_cam;
        pose_in.header.stamp = ros::Time(0);
        geometry_msgs::PoseStamped object_world;
        try {
            tfListener.transformPose("world", pose_in, object_world);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("Vision pick TF transform failed: %s", ex.what());
            return false;
        }

        // aruco_ros가 보고하는 pose는 "마커 패치"의 pose이지 박스 중심이 아니다.
        // aruco_box_26/model.sdf: 마커 패치를 박스 로컬 +Z면(윗면)에 pose z=+0.0505로 붙여뒀다.
        // 이 world의 aruco_box_26은 회전 없이(rpy=0) 스폰되므로 박스 중심 = 마커 위치 + (0,0,-0.0505).
        // (주의: 처음에는 pose의 orientation(Z축=마커 법선)으로 회전에 무관하게 일반화해서 보정하려 했으나,
        //  이 시야각(오블리크)에서는 ArUco의 orientation 추정 자체가 부정확해서 오히려 오차가 커짐을 실측으로
        //  확인함. position(위치) 추정은 안정적이므로, 이 데모 world처럼 물체가 항상 축정렬로 스폰되는
        //  경우엔 world-frame 고정 오프셋이 orientation 기반 보정보다 더 안정적이다.)
        // (이 보정 없이 마커 위치를 그대로 물체 중심으로 쓰면 grasp_offset=4.5cm 스퀴즈가 실제 박스 표면을
        //  몇 cm씩 빗나가 파지가 전혀 안 되는 문제가 있었음 - 실측으로 확인.)
        const double MARKER_TO_BOX_CENTER = 0.0505;

        Vector3d obj = Vector3d(object_world.pose.position.x,
                                 object_world.pose.position.y,
                                 object_world.pose.position.z)
                       + Vector3d(0, 0, -MARKER_TO_BOX_CENTER);
        Vector3d transport_pt(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);

        // 큐브 면 기준 파지 목표 생성:
        // - 좌/우 손은 큐브의 +Y / -Y face center를 향한다.
        // - 초기 contact는 face에서 약간 바깥쪽, final squeeze는 face 안쪽으로 소폭 침투시켜
        //   손바닥 면이 큐브 면에 맞닿은 뒤 더 조이도록 만든다.
        const double BOX_HALF_Y = 0.050;
        const double CONTACT_X_BIAS = -0.020;     // 몸쪽(-x)으로 더 당겨 손이 큐브 앞쪽이 아니라 옆면 중앙을 잡게 함
        const double CONTACT_Z_BIAS = -0.055;     // 중심보다 더 아래를 잡아 lift 때 받쳐들기 유리하게
        const double CONTACT_FACE_INSET = 0.008;  // 첫 접촉 시 face 안쪽 침투량 [m]
        const double SQUEEZE_FACE_INSET = 0.016;  // final squeeze 침투량 [m]

        VectorXd base_seed(DoF);
        for (int i = 0; i < DoF; i++) base_seed(i) = base_q[i];

        pinocchio::forwardKinematics(model, data, base_seed);
        pinocchio::updateFramePlacements(model, data);
        Vector3d start_L = data.oMf[l_EE].translation();
        Vector3d start_R = data.oMf[r_EE].translation();

        // 물체/이송목표의 좌우 face center 기반 목표점
        Vector3d objL = obj + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - CONTACT_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d objR = obj + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + CONTACT_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d squeezeL = obj + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d squeezeR = obj + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d transportL = transport_pt + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d transportR = transport_pt + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d releaseL = transport_pt + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y + 0.035, CONTACT_Z_BIAS);
        Vector3d releaseR = transport_pt + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y - 0.035, CONTACT_Z_BIAS);

        // pick_pedestal(world 파일)이 파지점 바로 아래(z 1.05~1.15)에 y로 걸쳐 있어서, 시작 자세에서
        // objL/R로 곧장 3D 직선 이동하면 z가 받침대 상판보다 낮은 구간에서 x,y가 이미 받침대 영역에
        // 들어가 팔이 모서리에 부딪힌다. 그래서 접근을 2단계로 나눈다: 먼저 파지 높이(obj.z, 받침대
        // 상판보다 5cm 위)를 유지한 채 받침대 바깥쪽으로 STANDOFF_Y만큼 더 벌어진 standoff 지점으로
        // 이동하고, 그다음 그 높이를 유지한 채 y 방향으로만 직선 이동해 파지점에 들어간다 - 마지막
        // 구간은 항상 받침대보다 높은 높이에서만 움직이므로 부딪힐 수 없다.
        const double STANDOFF_Y = 0.15;   // 받침대 y 반폭(0.06)보다 충분히 큰 여유
        const double STANDOFF_Z_LIFT = 0.03; // 첫 진입은 파지점보다 조금 더 높게 들어가 모서리 걸림 방지
        Vector3d standoffL = objL + Vector3d(0, STANDOFF_Y, STANDOFF_Z_LIFT);
        Vector3d standoffR = objR + Vector3d(0, -STANDOFF_Y, STANDOFF_Z_LIFT);

        // 파지 직후 곧바로 파지점->이송목표 대각선 직선으로 이동하면 받침대/바닥 근처를 스치듯 지나갈
        // 수 있다. 스퀴즈를 유지한 채(PHASE_GRASP_TO_PLACE) 먼저 수직으로 LIFT_HEIGHT만큼 들어올린 뒤,
        // 그 높이에서 이송목표로 이동한다.
        const double LIFT_HEIGHT = 0.08;  // 파지 높이에서 들어올릴 여유 [m]
        Vector3d liftL = squeezeL + Vector3d(0, 0, LIFT_HEIGHT);
        Vector3d liftR = squeezeR + Vector3d(0, 0, LIFT_HEIGHT);

        std::vector<MatrixXd> pos_segs, vel_segs, acc_segs;
        std::vector<int> seg_phase;   // 세그먼트별 TaskPhase 태그 (재생 중 자동 전환용)
        VectorXd seed_vec = base_seed;
        grasp_gate_row = -1;
        grasp_gate_end_row = -1;
        grasp_contact_ready = false;
        grasp_contact_ticks = 0;
        grasp_post_contact_hold_ticks = 0;

        // Cartesian 직선 구간 하나를 만들어 세그먼트 목록에 추가.
        // CartesianLineTrajectory로 6D(L+R) 직선 경로를 만들고, 매 웨이포인트마다 DLS IK를 풀어
        // 관절각 시퀀스로 변환한 뒤, 위치->속도->가속도를 중심차분으로 계산한다 (mode 2 cartesian sim과 동일 방식).
        auto addCartesianSegment = [&](const Vector3d& sL, const Vector3d& gL,
                                        const Vector3d& sR, const Vector3d& gR, int phase,
                                        double v_des = 0.1) {
            MatrixXd cart_p, cart_v, cart_a;
            dualarm.CartesianLineTrajectory(sL, gL, sR, gR, cart_p, cart_v, cart_a, v_des);
            int steps = cart_p.rows();

            MatrixXd jp(steps, DoF), jv(steps, DoF), ja(steps, DoF);

            // 1) 매 웨이포인트 IK -> 관절각 시퀀스
            for (int k = 0; k < steps; k++) {
                Vector3d pL(cart_p(k,0), cart_p(k,1), cart_p(k,2));
                Vector3d pR(cart_p(k,3), cart_p(k,4), cart_p(k,5));
                VectorXd q_k;
                dualarm.SolveIK_Position(model, data, l_EE, r_EE, pL, pR, seed_vec, q_k);
                for (int i = 0; i < DoF; i++) jp(k, i) = q_k(i);
                seed_vec = q_k;   // 다음 웨이포인트/다음 세그먼트로 시드 연속성 유지
            }

            // 2) 중심차분으로 속도 계산 (양 끝단은 전진/후진 차분)
            jv.setZero();
            for (int k = 1; k < steps - 1; k++)
                for (int i = 0; i < DoF; i++)
                    jv(k, i) = (jp(k+1, i) - jp(k-1, i)) / (2.0*SAMPLING_TIME_TRAJ);
            if (steps >= 2) {
                for (int i = 0; i < DoF; i++) {
                    jv(0, i)       = (jp(1, i) - jp(0, i)) / SAMPLING_TIME_TRAJ;
                    jv(steps-1, i) = (jp(steps-1, i) - jp(steps-2, i)) / SAMPLING_TIME_TRAJ;
                }
            }

            // 3) 속도를 다시 중심차분해서 가속도 계산
            ja.setZero();
            for (int k = 1; k < steps - 1; k++)
                for (int i = 0; i < DoF; i++)
                    ja(k, i) = (jv(k+1, i) - jv(k-1, i)) / (2.0*SAMPLING_TIME_TRAJ);

            pos_segs.push_back(jp);
            vel_segs.push_back(jv);
            acc_segs.push_back(ja);
            seg_phase.push_back(phase);
        };

        // ===== 전체 동작 순서 =====
        // 1) 팔을 물체 옆 standoff 지점(파지 높이 유지, 받침대 바깥쪽)으로 이동
        addCartesianSegment(start_L, standoffL, start_R, standoffR, PHASE_APPROACH);

        // 1b) standoff -> 파지 위치로 y 방향 직선 접근 (파지 높이를 그대로 유지하므로 받침대와 부딪히지 않음)
        // objL/R은 이미 박스 표면 안쪽(grasp_offset 침투)까지를 목표로 하므로, 기본 속도(0.1m/s)로
        // 그대로 들어가면 접촉 순간 충격이 커서 좌우 접촉이 어긋나며 물체가 회전하며 떨어지는 문제가
        // 있었다 - 이 구간만 더 느리게 접근해 접촉 충격을 줄인다.
        const double APPROACH_CONTACT_V_DES = 0.02;
        addCartesianSegment(standoffL, objL, standoffR, objR, PHASE_APPROACH, APPROACH_CONTACT_V_DES);

        // 2) 첫 접촉 후, lift 전에 짧은 추가 압착으로 손끝 판이 더 면접촉에 가까워지도록 만든다.
        const double FINAL_SQUEEZE_V_DES = 0.01;
        addCartesianSegment(objL, squeezeL, objR, squeezeR, PHASE_GRASP_TO_PLACE, FINAL_SQUEEZE_V_DES);
        {
            int rows_before_lift = 0;
            for (const auto& seg : pos_segs) rows_before_lift += seg.rows();
            grasp_gate_row = rows_before_lift;
        }

        // 3) 추가 압착 상태에서 수직으로 들어올리기. 이 구간부터 어드미턴스가 계속 켜진 상태다.
        addCartesianSegment(squeezeL, liftL, squeezeR, liftR, PHASE_GRASP_TO_PLACE, 0.025);

        // 4) 들어올린 높이를 유지한 채 목표 지점으로 이동 (계속 PHASE_GRASP_TO_PLACE, 스퀴즈 유지)
        addCartesianSegment(liftL, transportL, liftR, transportR, PHASE_GRASP_TO_PLACE, 0.035);
        {
            int rows_before_return = 0;
            for (const auto& seg : pos_segs) rows_before_return += seg.rows();
            grasp_gate_end_row = rows_before_return;
        }

        // 5) 목표 지점에서 양팔을 바깥으로 벌려 물체를 놓는다.
        addCartesianSegment(transportL, releaseL, transportR, releaseR, PHASE_RETURN, 0.02);

        // 6) 내려놓기 완료 -> 원래 위치로 복귀
        addCartesianSegment(releaseL, start_L, releaseR, start_R, PHASE_RETURN);

        ROS_INFO("Vision pick(dual-arm): object(world)=[%.3f %.3f %.3f], transport=[%.3f %.3f %.3f]",
                 obj.x(), obj.y(), obj.z(),
                 transport_pt.x(), transport_pt.y(), transport_pt.z());
        for (size_t k = 0; k < pos_segs.size(); ++k) {
            ROS_INFO("  segment[%zu]: rows=%d phase=%d", k, (int)pos_segs[k].rows(), seg_phase[k]);
        }

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

        traj_cnt = 0;
        traj_done_published = false;
        return true;
    };

    // ===== Head 스캔 상태머신 진행 (현재 웨이포인트 이동/재생이 끝난 뒤 매 tick 호출) =====
    auto runScanStep = [&]() {
        if (scan_step == SCAN_MOVE) {
            // 방금 웨이포인트 이동이 끝남 -> 정지 대기(SETTLE) 시작
            scan_step = SCAN_SETTLE;
            scan_wait_cnt = 0;
        }
        else if (scan_step == SCAN_SETTLE) {
            scan_wait_cnt++;
            if (scan_wait_cnt >= SCAN_SETTLE_TICKS) {
                scan_step = SCAN_CHECK;
                scan_wait_cnt = 0;
                scan_match_buf.clear();
                scan_last_seq = aruco_pose_seq;   // 이 시점 이후 도착하는 프레임만 카운트
            }
        }
        else if (scan_step == SCAN_CHECK) {
            if (aruco_pose_received && aruco_pose_seq != scan_last_seq) {
                scan_last_seq = aruco_pose_seq;
                Vector3d p(aruco_pose_cam.pose.position.x,
                           aruco_pose_cam.pose.position.y,
                           aruco_pose_cam.pose.position.z);
                if (!scan_match_buf.empty() && (p - scan_match_buf.back()).norm() > SCAN_MATCH_TOL) {
                    scan_match_buf.clear();   // 직전 프레임과 불일치 -> 처음부터 다시 셈
                }
                scan_match_buf.push_back(p);
            }

            if ((int)scan_match_buf.size() >= SCAN_MATCH_FRAMES) {
                ROS_INFO("Head scan: marker confirmed (yaw=%.1fdeg, pitch=%.1fdeg)",
                         HEAD_SCAN_YAW * rad2deg,
                         HEAD_SCAN_PITCH_CANDIDATES[scan_pitch_index] * rad2deg);
                scan_active = false;
                if (!buildGraspPipelineFromDetection(scan_q)) {
                    task_phase = PHASE_APPROACH;   // TF 실패 -> 실패 처리, 접근 단계로 리셋
                }
                return;
            }

            scan_wait_cnt++;
            if (scan_wait_cnt >= SCAN_CHECK_TIMEOUT_TICKS) {
                const double failed_pitch = HEAD_SCAN_PITCH_CANDIDATES[scan_pitch_index];
                ROS_WARN("Head scan: object not found at scan pose (yaw=%.1fdeg, pitch=%.1fdeg).",
                         HEAD_SCAN_YAW * rad2deg, failed_pitch * rad2deg);
                if (!advanceScanPose()) {
                    scan_active = false;
                    task_phase = PHASE_APPROACH;   // 실패 처리: 접근 단계로 리셋, 마지막 자세에서 정지 유지
                }
            }
        }
    };

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

        const bool all_joint_states_received =
            waist_state_received && head_state_received && left_arm_state_received && right_arm_state_received;

        if (startup_state != STARTUP_COMPLETE) {
            if (!all_joint_states_received) {
                loop_rate.sleep();
                ros::spinOnce();
                continue;
            }

            if (startup_state == STARTUP_WAIT_FOR_STATE) {
                startup_hold_q[0] = waist_jointp[0];
                startup_hold_q[1] = head_jointp[0];
                startup_hold_q[2] = head_jointp[1];
                for (int i = 0; i < 4; ++i) {
                    startup_hold_q[i + 3] = left_arm_jointp[i];
                    startup_hold_q[i + 7] = right_arm_jointp[i];
                }

                dual_arm_jointp_trajectory.resize(1, DoF);
                dual_arm_jointv_trajectory.resize(1, DoF);
                dual_arm_jointa_trajectory.resize(1, DoF);
                dual_arm_phase_trajectory.resize(1);
                for (int i = 0; i < DoF; ++i) {
                    dual_arm_jointp_trajectory(0, i) = startup_hold_q[i];
                    dual_arm_jointv_trajectory(0, i) = 0.0;
                    dual_arm_jointa_trajectory(0, i) = 0.0;
                }
                dual_arm_phase_trajectory(0) = PHASE_APPROACH;
                traj_cnt = 0;
                startup_hold_cnt = 0;
                startup_state = STARTUP_HOLD_CURRENT;
                ROS_INFO("Startup: captured current joint state, holding to let the robot settle.");
            }
            else if (startup_state == STARTUP_HOLD_CURRENT) {
                double joint_vel_norm = fabs(waist_jointv[0]) + fabs(head_jointv[0]) + fabs(head_jointv[1]);
                for (int i = 0; i < 4; ++i) {
                    joint_vel_norm += fabs(left_arm_jointv[i]) + fabs(right_arm_jointv[i]);
                }

                if (joint_vel_norm < STARTUP_SETTLE_VEL_NORM) {
                    startup_hold_cnt++;
                } else {
                    startup_hold_cnt = 0;
                }

                if (startup_hold_cnt >= STARTUP_HOLD_TICKS) {
                    buildFixedDurationJointTrajectory(startup_hold_q, startup_target_q, STARTUP_MOVE_DURATION);
                    traj_cnt = 0;
                    traj_done_published = false;
                    startup_state = STARTUP_MOVE_TO_TARGET;
                    ROS_INFO("Startup: moving to elbow-down ready pose over %.1f s.", STARTUP_MOVE_DURATION);
                }
            }
            else if (startup_state == STARTUP_MOVE_TO_TARGET && traj_cnt >= dual_arm_jointp_trajectory.rows()) {
                startup_state = STARTUP_COMPLETE;
                ROS_INFO("Startup: ready pose reached, enabling normal command processing.");
            }
        }
        VectorXd nominal_joint_acc_vec = VectorXd::Zero(DoF);

        if(startup_state == STARTUP_COMPLETE && callback == true){
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
                // 수동 modeling 명령은 어드미턴스 대상이 아님 -> 전 구간 접근 단계로 태깅
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
                // 수동 joint sim 명령은 어드미턴스 대상이 아님 -> 전 구간 접근 단계로 태깅
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

                // 수동 cartesian sim 명령은 어드미턴스 대상이 아님 -> 전 구간 접근 단계로 태깅
                dual_arm_phase_trajectory.setConstant(steps, PHASE_APPROACH);
            }

            // ===== 모드 3: vision pick (Head 이동 -> ArUco 검출 -> world 변환 -> 파지/이송/복귀) =====
            // 새 vision pick 명령이 들어올 때마다 항상 검출부터 새로 시작한다 - 이전에 우연히 잡혔을 수
            // 있는 오래된 aruco_pose_cam을 그대로 재사용하지 않기 위해 aruco_pose_received를 강제로 리셋한다.
            // (그 다음 실제 파지 파이프라인 생성은 검출을 확정한 뒤 runScanStep()에서 수행된다.)
            else if (command_mode == 3) {
                double tx = dual_arm_commandx[0], ty = dual_arm_commandx[1], tz = dual_arm_commandx[2];

                aruco_pose_received = false;
                scan_match_buf.clear();
                scan_pitch_index = 0;
                for (int i = 0; i < DoF; i++) scan_q[i] = dual_arm_initp[i];

                // 이번 명령의 이송 목표 위치에 표시/받침대(place_indicator)를 스폰 - 스캔/파지가
                // 끝나기 한참 전인 지금 미리 만들어둬야, 이후 이송 중에도 목표 지점을 눈으로 계속 볼 수 있다.
                spawnPlaceIndicator(Vector3d(tx, ty, tz));

                ROS_INFO("Vision pick: moving head to scan pose (yaw=%.1fdeg, pitch=%.1fdeg) and checking for marker.",
                         HEAD_SCAN_YAW * rad2deg,
                         HEAD_SCAN_PITCH_CANDIDATES[scan_pitch_index] * rad2deg);

                startScanMoveTo(HEAD_SCAN_YAW, HEAD_SCAN_PITCH_CANDIDATES[scan_pitch_index]);
                scan_active = true;
            }

            if (new_trajectory_built) {
                traj_cnt = 0;
                traj_done_published = false;   // 새 궤적 시작 -> 완료 알림 다시 대기
            }
            callback = false;
        }
        else if (traj_cnt < dual_arm_jointp_trajectory.rows()){
            if (grasp_gate_row > 0 &&
                traj_cnt >= grasp_gate_row &&
                (grasp_gate_end_row < 0 || traj_cnt < grasp_gate_end_row)) {
                if (!grasp_contact_ready) {
                    traj_cnt = grasp_gate_row - 1;  // 마지막 squeeze row에 고정, bilateral contact 전에는 lift 금지
                } else if (grasp_post_contact_hold_ticks < GRASP_POST_CONTACT_HOLD_TICKS) {
                    traj_cnt = grasp_gate_row - 1;  // 접촉 직후 그대로 더 조여서 면접촉을 안정화
                    grasp_post_contact_hold_ticks++;
                }
            }
            for (int i = 0; i < DoF; i++){
                dual_arm_targetp[i] = dual_arm_jointp_trajectory(traj_cnt, i);
            }
            for (int i = 0; i < DoF; i++){
                dual_arm_targetv[i] = dual_arm_jointv_trajectory(traj_cnt, i);
            }
            for (int i = 0; i < DoF; i++){
                nominal_joint_acc_vec(i) = dual_arm_jointa_trajectory(traj_cnt, i);
            }

            // vision pick(mode 3) 재생 중이면 현재 행에 태깅된 phase로 자동 전환.
            // (mode 0/1/2는 전 구간 PHASE_APPROACH로 태깅되어 있어 어드미턴스가 자동으로 켜지지 않음)
            if (traj_cnt < dual_arm_phase_trajectory.size()) {
                const int next_phase = dual_arm_phase_trajectory(traj_cnt);
                if (next_phase != task_phase) {
                    ROS_INFO("Phase switch by trajectory: traj_cnt=%d -> phase=%d", traj_cnt, next_phase);
                }
                task_phase = next_phase;
            }

            traj_cnt++;
        }
        else {
            // 마지막 타겟 자세 유지
            for (int i = 0; i < DoF; i++) {
                dual_arm_targetp[i] = dual_arm_jointp_trajectory.bottomRows(1)(0, i);
                dual_arm_targetv[i] = 0.0;  // 정지 목표
            }

            if (scan_active) {
                // Head 자동 스캔 진행 중: 방금 웨이포인트 이동이 끝났거나(SCAN_MOVE),
                // 정지 대기(SCAN_SETTLE) 또는 인식 확인(SCAN_CHECK) 중 -> 매 tick 상태머신 진행.
                // 검출이 확정되면 이 안에서 파지 파이프라인이 새로 만들어져 traj_cnt가 리셋되므로,
                // 다음 tick부터는 이 else 분기가 아니라 위쪽 재생 분기가 자동으로 이어받는다.
                runScanStep();
            }
            else if (!traj_done_published) {
                // 궤적(복귀 포함) 실행이 막 끝난 시점: 접근 단계로 리셋 + 완료 알림, 딱 한 번만
                // (스캔 실패로 여기 들어온 경우도 포함 - runScanStep()이 scan_active를 false로 내리고
                //  task_phase를 PHASE_APPROACH로 리셋한 뒤 다음 tick에 이 분기로 자연스럽게 넘어온다)
                task_phase = PHASE_APPROACH;   // 복귀 완료 -> 접근 단계로 리셋 (어드미턴스 OFF)

                std_msgs::Bool traj_done_msg;
                traj_done_msg.data = true;
                dual_armtraj_done_pub.publish(traj_done_msg);
                traj_done_published = true;
                if (command_mode == 3) {
                    command_mode = -1;  // 완료된 vision pick을 자동으로 다시 시작하지 않음
                }
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
        VectorXd dual_arm_targetv_vec = VectorXd::Zero(DoF);
        for (int i = 0; i < DoF; ++i) {
            dual_arm_targetv_vec(i) = dual_arm_targetv[i];
        }

        // ===== 어드미턴스 제어 (파지~내려놓기 구간에서만 활성화) =====
        // Ma*x_ddot + Da*x_dot + Ka*x = F_ext 를 적분해 Cartesian compliance offset을 만들고,
        // 그 오프셋을 arm-only Jacobian으로 관절 목표 위치/속도에 반영한다.
        bool admittance_active = (task_phase == PHASE_GRASP_TO_PLACE);
        if (admittance_active) {
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
            MatrixXd JL_arm(3, 4);
            MatrixXd JR_arm(3, 4);
            for (int c = 0; c < 4; ++c) {
                JL_arm.col(c) = JL.col(c + 3);
                JR_arm.col(c) = JR.col(c + 7);
            }

            // nominal target EE 위치 (target 궤적 기준 FK)
            pinocchio::forwardKinematics(model, data, dual_arm_targetp_vec);
            pinocchio::updateFramePlacements(model, data);
            // F/T 원시값은 접촉 순간 노이즈가 커서 그대로 적분하면 순응 상태가 요동하므로 로우패스로 완화.
            for (int k = 0; k < 3; k++) {
                left_ft_force_lpf(k)  = dualarm.LowPassFilter(left_ft_force(k),  left_ft_force_before(k),  FT_LPF_CUTOFF_HZ);
                right_ft_force_lpf(k) = dualarm.LowPassFilter(right_ft_force(k), right_ft_force_before(k), FT_LPF_CUTOFF_HZ);
                left_ft_torque_lpf(k)  = dualarm.LowPassFilter(left_ft_torque(k),  left_ft_torque_before(k),  FT_LPF_CUTOFF_HZ);
                right_ft_torque_lpf(k) = dualarm.LowPassFilter(right_ft_torque(k), right_ft_torque_before(k), FT_LPF_CUTOFF_HZ);
            }
            left_ft_force_before  = left_ft_force_lpf;
            right_ft_force_before = right_ft_force_lpf;
            left_ft_torque_before  = left_ft_torque_lpf;
            right_ft_torque_before = right_ft_torque_lpf;

            // F/T 센서 힘: 센서가 EE 프레임과 동일 방향으로 장착되었다고 가정하고 world frame으로 변환
            Vector3d F_ext_L = RL_actual * left_ft_force_lpf;
            Vector3d F_ext_R = RR_actual * right_ft_force_lpf;
            if (!grasp_contact_ready) {
                const bool left_contact_ok = std::abs(F_ext_L(1)) >= GRASP_CONTACT_FORCE_THRESHOLD;
                const bool right_contact_ok = std::abs(F_ext_R(1)) >= GRASP_CONTACT_FORCE_THRESHOLD;
                const bool left_face_ok =
                    std::abs(left_ft_torque_lpf(0)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD &&
                    std::abs(left_ft_torque_lpf(2)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD;
                const bool right_face_ok =
                    std::abs(right_ft_torque_lpf(0)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD &&
                    std::abs(right_ft_torque_lpf(2)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD;
                if (left_contact_ok && right_contact_ok && left_face_ok && right_face_ok) {
                    grasp_contact_ticks++;
                    if (grasp_contact_ticks >= GRASP_CONTACT_HOLD_TICKS) {
                        grasp_contact_ready = true;
                        ROS_INFO("Grasp face-contact ready: |Fy_L|=%.2f N |Fy_R|=%.2f N, |tau_xz_L|=[%.3f %.3f], |tau_xz_R|=[%.3f %.3f]",
                                 std::abs(F_ext_L(1)), std::abs(F_ext_R(1)),
                                 std::abs(left_ft_torque_lpf(0)), std::abs(left_ft_torque_lpf(2)),
                                 std::abs(right_ft_torque_lpf(0)), std::abs(right_ft_torque_lpf(2)));
                    }
                } else {
                    grasp_contact_ticks = 0;
                    grasp_post_contact_hold_ticks = 0;
                }
            }

            Vector3d xL_adm_ddot = Vector3d::Zero();
            Vector3d xR_adm_ddot = Vector3d::Zero();
            Vector3d F_ctrl_L = -F_ext_L;
            Vector3d F_ctrl_R = -F_ext_R;
            F_ctrl_L(1) = F_ext_L(1) - DESIRED_SQUEEZE_FORCE;
            F_ctrl_R(1) = F_ext_R(1) + DESIRED_SQUEEZE_FORCE;
            for (int k = 0; k < 3; k++) {
                xL_adm_ddot(k) = (F_ctrl_L(k) - Da_left[k] * left_adm_vel(k) - Ka_left[k] * left_adm_pos(k)) / Ma_left[k];
                xR_adm_ddot(k) = (F_ctrl_R(k) - Da_right[k] * right_adm_vel(k) - Ka_right[k] * right_adm_pos(k)) / Ma_right[k];

                left_adm_vel(k) += xL_adm_ddot(k) * SAMPLING_TIME;
                right_adm_vel(k) += xR_adm_ddot(k) * SAMPLING_TIME;

                left_adm_vel(k) = std::max(-ADMITTANCE_VEL_LIMIT, std::min(ADMITTANCE_VEL_LIMIT, left_adm_vel(k)));
                right_adm_vel(k) = std::max(-ADMITTANCE_VEL_LIMIT, std::min(ADMITTANCE_VEL_LIMIT, right_adm_vel(k)));

                left_adm_pos(k) += left_adm_vel(k) * SAMPLING_TIME;
                right_adm_pos(k) += right_adm_vel(k) * SAMPLING_TIME;

                left_adm_pos(k) = std::max(-ADMITTANCE_POS_LIMIT, std::min(ADMITTANCE_POS_LIMIT, left_adm_pos(k)));
                right_adm_pos(k) = std::max(-ADMITTANCE_POS_LIMIT, std::min(ADMITTANCE_POS_LIMIT, right_adm_pos(k)));
            }

            VectorXd dq_adm_L = dualarm.DampedPinv(JL_arm, ADMITTANCE_DLS_LAMBDA) * left_adm_pos;
            VectorXd dq_adm_R = dualarm.DampedPinv(JR_arm, ADMITTANCE_DLS_LAMBDA) * right_adm_pos;
            VectorXd dq_adm_dot_L = dualarm.DampedPinv(JL_arm, ADMITTANCE_DLS_LAMBDA) * left_adm_vel;
            VectorXd dq_adm_dot_R = dualarm.DampedPinv(JR_arm, ADMITTANCE_DLS_LAMBDA) * right_adm_vel;

            for (int i = 0; i < 4; ++i) {
                dual_arm_targetp_vec(i + 3) += dq_adm_L(i);
                dual_arm_targetp_vec(i + 7) += dq_adm_R(i);
                dual_arm_targetv_vec(i + 3) += dq_adm_dot_L(i);
                dual_arm_targetv_vec(i + 7) += dq_adm_dot_R(i);
            }

            for (int i = 0; i < DoF; ++i) {
                dual_arm_targetp[i] = dual_arm_targetp_vec(i);
                dual_arm_targetv[i] = dual_arm_targetv_vec(i);
            }

        }
        else {
            left_adm_pos.setZero();
            left_adm_vel.setZero();
            right_adm_pos.setZero();
            right_adm_vel.setZero();
            grasp_contact_ticks = 0;
            grasp_post_contact_hold_ticks = 0;
            if (task_phase != PHASE_GRASP_TO_PLACE) {
                grasp_contact_ready = false;
            }
        }

        dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
        for (int i = 0; i < DoF; i++) {
            dual_arm_targeta_vec(i) = nominal_joint_acc_vec(i) + PD_acc[i];
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
        

        //cout << "=== Left End-Effector Pose ===" << endl;
        //cout << "Position: " << data.oMf[l_EE].translation().transpose() << endl;
        //cout << "Orientation (Rotation Matrix):\n" << data.oMf[l_EE].rotation() << endl;
        
        //cout << "=== Right End-Effector Pose ===" << endl;
        //cout << "Position: " << data.oMf[r_EE].translation().transpose() << endl;
        //cout << "Orientation (Rotation Matrix):\n" << data.oMf[r_EE].rotation() << endl;     

        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
