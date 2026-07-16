#include "dual_arm_function.cpp"
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <vector>
#include <algorithm>
#include <limits>
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
        <pose>0 0 -0.15 0 0 0</pose>
        <geometry><box><size>0.12 0.36 0.10</size></box></geometry>
      </collision>
      <visual name="pedestal_visual">
        <pose>0 0 -0.15 0 0 0</pose>
        <geometry><box><size>0.12 0.36 0.10</size></box></geometry>
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
const double STARTUP_L_SHOULDER_PITCH = 0.0;
const double STARTUP_L_SHOULDER_ROLL = 0.0;
const double STARTUP_L_SHOULDER_YAW = 0.0;
const double STARTUP_L_ELBOW = 0.0;
const double STARTUP_L_WRIST_YAW = 0.0;
const double STARTUP_R_SHOULDER_PITCH = 0.0;
const double STARTUP_R_SHOULDER_ROLL = 0.0;
const double STARTUP_R_SHOULDER_YAW = 0.0;
const double STARTUP_R_ELBOW = 0.0;
const double STARTUP_R_WRIST_YAW = 0.0;
const int STARTUP_HOLD_TICKS = 1000;           // 1.0s @ 1000Hz
const double STARTUP_MOVE_DURATION = 3.0;      // [s] 초기 자세로 천천히 이동
const double STARTUP_SETTLE_VEL_NORM = 0.35;   // [rad/s]

double finiteOrZero(double value)
{
    return std::isfinite(value) ? value : 0.0;
}

double wrapToJointRange(double value, double lower, double upper)
{
    if (!std::isfinite(value)) {
        return 0.0;
    }

    const double two_pi = 2.0 * M_PI;
    const double mid = 0.5 * (lower + upper);
    double best = value;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int k = -2; k <= 2; ++k) {
        double cand = value + k * two_pi;
        double range_error = 0.0;
        if (cand < lower) range_error = lower - cand;
        else if (cand > upper) range_error = cand - upper;

        const double center_error = std::abs(cand - mid);
        const double cost = 100.0 * range_error + center_error;
        if (cost < best_cost) {
            best_cost = cost;
            best = cand;
        }
    }

    return best;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
// /dual_arm/joint_states 의 name 배열은 조인트 이름 알파벳순으로 정렬되어 온다(실측 확인).
// Head_pitch_joint(0) < Head_yaw_joint(1) < L_elbow_joint(2) < L_shoulder_pitch_joint(3) <
// L_shoulder_roll_joint(4) < L_shoulder_yaw_joint(5) < L_wrist_yaw_joint(6) <
// R_elbow_joint(7) < R_shoulder_pitch_joint(8) < R_shoulder_roll_joint(9) < R_shoulder_yaw_joint(10) <
// R_wrist_yaw_joint(11) < Waist_joint(12)
void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        waist_state_received = true;
        waist_jointp[0] = wrapToJointRange(msg->position[12], -0.87, 0.87);
        waist_jointv[0] = finiteOrZero(msg->velocity[12]);
        waist_torque[0] = finiteOrZero(msg->effort[12]);
    }

void msgCallbackHeadJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        head_state_received = true;
        head_jointp[0] = wrapToJointRange(msg->position[1], -1.2217, 1.2217);   // yaw
        head_jointv[0] = finiteOrZero(msg->velocity[1]);
        head_torque[0] = finiteOrZero(msg->effort[1]);

        head_jointp[1] = wrapToJointRange(msg->position[0], -0.5236, 1.0472);   // pitch
        head_jointv[1] = finiteOrZero(msg->velocity[0]);
        head_torque[1] = finiteOrZero(msg->effort[0]);
    }

void msgCallbackLeftArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        left_arm_state_received = true;
        left_arm_jointp[0] = wrapToJointRange(msg->position[3], -3.14, 1.05);
        left_arm_jointv[0] = finiteOrZero(msg->velocity[3]);
        left_arm_torque[0] = finiteOrZero(msg->effort[3]);

        left_arm_jointp[1] = wrapToJointRange(msg->position[4], -0.349, 2.62);
        left_arm_jointv[1] = finiteOrZero(msg->velocity[4]);
        left_arm_torque[1] = finiteOrZero(msg->effort[4]);

        left_arm_jointp[2] = wrapToJointRange(msg->position[5], -1.57, 1.57);
        left_arm_jointv[2] = finiteOrZero(msg->velocity[5]);
        left_arm_torque[2] = finiteOrZero(msg->effort[5]);

        left_arm_jointp[3] = wrapToJointRange(msg->position[2], -1.9, 1.05);
        left_arm_jointv[3] = finiteOrZero(msg->velocity[2]);
        left_arm_torque[3] = finiteOrZero(msg->effort[2]);

        left_arm_jointp[4] = wrapToJointRange(msg->position[6], -0.9, 0.9);
        left_arm_jointv[4] = finiteOrZero(msg->velocity[6]);
        left_arm_torque[4] = finiteOrZero(msg->effort[6]);
    }

void msgCallbackRightArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        right_arm_state_received = true;
        right_arm_jointp[0] = wrapToJointRange(msg->position[8], -3.14, 1.05);
        right_arm_jointv[0] = finiteOrZero(msg->velocity[8]);
        right_arm_torque[0] = finiteOrZero(msg->effort[8]);

        right_arm_jointp[1] = wrapToJointRange(msg->position[9], -2.62, 0.349);
        right_arm_jointv[1] = finiteOrZero(msg->velocity[9]);
        right_arm_torque[1] = finiteOrZero(msg->effort[9]);

        right_arm_jointp[2] = wrapToJointRange(msg->position[10], -1.57, 1.57);
        right_arm_jointv[2] = finiteOrZero(msg->velocity[10]);
        right_arm_torque[2] = finiteOrZero(msg->effort[10]);

        right_arm_jointp[3] = wrapToJointRange(msg->position[7], -1.9, 1.05);
        right_arm_jointv[3] = finiteOrZero(msg->velocity[7]);
        right_arm_torque[3] = finiteOrZero(msg->effort[7]);

        right_arm_jointp[4] = wrapToJointRange(msg->position[11], -0.9, 0.9);
        right_arm_jointv[4] = finiteOrZero(msg->velocity[11]);
        right_arm_torque[4] = finiteOrZero(msg->effort[11]);
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
        ros::Publisher dual_armjoint12_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint12_position_controller/command", 100);
        ros::Publisher dual_armjoint13_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint13_position_controller/command", 100);

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
        ros::Publisher dual_armjoint12_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint12_effort_controller/command", 100);
        ros::Publisher dual_armjoint13_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint13_effort_controller/command", 100);

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
    ros::Publisher left_grasp_force_pub = nh.advertise<std_msgs::Float64>("/dual_arm/grasp_force_left", 20);
    ros::Publisher right_grasp_force_pub = nh.advertise<std_msgs::Float64>("/dual_arm/grasp_force_right", 20);
    ros::Publisher grasp_force_target_pub = nh.advertise<std_msgs::Float64>("/dual_arm/grasp_force_target", 20);
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

    ros::WallRate loop_rate(1000);
    ros::spinOnce();
    
    // 조인트 명령 메시지 객체 선언
    std_msgs::Float64 waist_joint_msg;
    std_msgs::Float64 head_yaw_joint_msg, head_pitch_joint_msg;
    std_msgs::Float64 shoulder_pitch_l_joint_msg, shoulder_roll_l_joint_msg, shoulder_yaw_l_joint_msg, elbow_l_joint_msg, wrist_yaw_l_joint_msg;
    std_msgs::Float64 shoulder_pitch_r_joint_msg, shoulder_roll_r_joint_msg, shoulder_yaw_r_joint_msg, elbow_r_joint_msg, wrist_yaw_r_joint_msg;

    // 고정된 URDF 파일 경로 사용
    string urdf_filename = ros::package::getPath("dual_arm") + "/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // IK는 wrist 아래 frame으로 arm pose를 풀고, grasp/contact 목표와 Jacobian은
    // collision pad의 실제 중심 frame을 사용한다.
    pinocchio::FrameIndex l_ik_EE = model.getFrameId("L_wrist_ik_joint");
    pinocchio::FrameIndex r_ik_EE = model.getFrameId("R_wrist_ik_joint");
    pinocchio::FrameIndex l_nominal_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_nominal_EE = model.getFrameId("R_EE_joint");
    pinocchio::FrameIndex l_contact_EE = model.getFrameId("L_grip_joint");
    pinocchio::FrameIndex r_contact_EE = model.getFrameId("R_grip_joint");

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
        STARTUP_L_WRIST_YAW,
        STARTUP_R_SHOULDER_PITCH,
        STARTUP_R_SHOULDER_ROLL,
        STARTUP_R_SHOULDER_YAW,
        STARTUP_R_ELBOW,
        STARTUP_R_WRIST_YAW;

    enum StartupState { STARTUP_WAIT_FOR_STATE, STARTUP_MOVE_TO_TARGET, STARTUP_COMPLETE };
    StartupState startup_state = STARTUP_WAIT_FOR_STATE;
    bool startup_zero_state_assumed = false;
    double startup_hold_q[DoF] = {0.0,};
    double startup_target_q[DoF] = {
        STARTUP_WAIST,
        STARTUP_HEAD_YAW,
        STARTUP_HEAD_PITCH,
        STARTUP_L_SHOULDER_PITCH,
        STARTUP_L_SHOULDER_ROLL,
        STARTUP_L_SHOULDER_YAW,
        STARTUP_L_ELBOW,
        STARTUP_L_WRIST_YAW,
        STARTUP_R_SHOULDER_PITCH,
        STARTUP_R_SHOULDER_ROLL,
        STARTUP_R_SHOULDER_YAW,
        STARTUP_R_ELBOW,
        STARTUP_R_WRIST_YAW
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

    auto mapContactTargetsToIkTargets = [&](const VectorXd& q_ref,
                                           const Vector3d& contact_target_L, const Vector3d& contact_target_R,
                                           Vector3d& ik_target_L, Vector3d& ik_target_R) {
        pinocchio::forwardKinematics(model, data, q_ref);
        pinocchio::updateFramePlacements(model, data);

        // Nominal trajectories remain wrist/EE based.  The grip-pad frame is
        // reserved for contact sensing and admittance after squeeze begins.
        const Vector3d left_nominal_offset =
            data.oMf[l_nominal_EE].translation() - data.oMf[l_ik_EE].translation();
        const Vector3d right_nominal_offset =
            data.oMf[r_nominal_EE].translation() - data.oMf[r_ik_EE].translation();

        ik_target_L = contact_target_L - left_nominal_offset;
        ik_target_R = contact_target_R - right_nominal_offset;
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
        const double MARKER_TO_BOX_CENTER = 0.1005;

        Vector3d obj = Vector3d(object_world.pose.position.x,
                                 object_world.pose.position.y,
                                 object_world.pose.position.z)
                       + Vector3d(0, 0, -MARKER_TO_BOX_CENTER);
        obj.y() += 0.023;  // 반복 측정된 ArUco world-Y 편향(-2.3cm) 보정
        Vector3d transport_pt(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);

        // 큐브 면 기준 파지 목표 생성:
        // - 좌/우 손은 큐브의 +Y / -Y face center를 향한다.
        // - 초기 contact는 face에서 약간 바깥쪽, final squeeze는 face 안쪽으로 소폭 침투시켜
        //   손바닥 면이 큐브 면에 맞닿은 뒤 더 조이도록 만든다.
        // - startup은 차렷으로 유지하고, 실제 접근은 먼저 큐브 위 high pregrasp로 올린 뒤
        //   수직 하강으로 바꿔 초기 raised-arm 불안정을 피한다.
        const double BOX_HALF_Y = 0.160;
        const double CONTACT_X_BIAS = -0.020;     // 몸쪽(-x)으로 더 당겨 손이 큐브 앞쪽이 아니라 옆면 중앙을 잡게 함
        // 박스 중심 높이에 패드 중심을 맞춘다. 패드 높이는 9 cm이고 받침대
        // 상단은 박스 바닥과 같으므로, 이전 -7 cm 목표는 패드 하단이
        // 받침대를 2.6 cm 관통해 F/T 센서가 박스 대신 받침대 반력을 읽었다.
        const double CONTACT_Z_BIAS = 0.000;
        const double CONTACT_FACE_INSET = 0.008;  // 첫 접촉 시 face 안쪽 침투량 [m]
        const double SQUEEZE_FACE_INSET = 0.028;  // 양쪽 grip pad가 큐브 면에 확실히 닿도록 손당 12mm 추가 squeeze
        const double ARM_RAISE_Z = 0.16;          // 차렷 후 먼저 제자리에서 들어올릴 높이 [m]
        const double ARM_RAISE_OUTWARD_Y = 0.05;  // 들어올릴 때 몸통과 팔 간섭을 줄이기 위한 바깥쪽 여유 [m]
        const double PREGRASP_HIGH_Z = 0.18;      // 큐브 위쪽 safe pregrasp 높이 [m]
        const double PREGRASP_WIDE_Y = 0.16;      // 넓어진 박스 접근 전 양팔을 더 벌림 [m]

        VectorXd base_seed(DoF);
        for (int i = 0; i < DoF; i++) base_seed(i) = base_q[i];

        pinocchio::forwardKinematics(model, data, base_seed);
        pinocchio::updateFramePlacements(model, data);
        Vector3d start_L = data.oMf[l_contact_EE].translation();
        Vector3d start_R = data.oMf[r_contact_EE].translation();

        // 물체/이송목표의 좌우 face center 기반 목표점
        Vector3d objL = obj + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - CONTACT_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d objR = obj + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + CONTACT_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d squeezeL = obj + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d squeezeR = obj + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d armRaiseL = start_L + Vector3d(0,  ARM_RAISE_OUTWARD_Y, ARM_RAISE_Z);
        Vector3d armRaiseR = start_R + Vector3d(0, -ARM_RAISE_OUTWARD_Y, ARM_RAISE_Z);
        Vector3d pregraspHighWideL = objL + Vector3d(0,  PREGRASP_WIDE_Y, PREGRASP_HIGH_Z);
        Vector3d pregraspHighWideR = objR + Vector3d(0, -PREGRASP_WIDE_Y, PREGRASP_HIGH_Z);
        Vector3d pregraspHighAlignL = objL + Vector3d(0, 0, PREGRASP_HIGH_Z);
        Vector3d pregraspHighAlignR = objR + Vector3d(0, 0, PREGRASP_HIGH_Z);
        Vector3d transportL = transport_pt + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y - SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d transportR = transport_pt + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y + SQUEEZE_FACE_INSET, CONTACT_Z_BIAS);
        Vector3d releaseL = transport_pt + Vector3d(CONTACT_X_BIAS,  BOX_HALF_Y + 0.035, CONTACT_Z_BIAS);
        Vector3d releaseR = transport_pt + Vector3d(CONTACT_X_BIAS, -BOX_HALF_Y - 0.035, CONTACT_Z_BIAS);

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
        grasp_acquired_once = false;
        grasp_contact_ticks = 0;
        grasp_post_contact_hold_ticks = 0;
        left_adm_recenter_count = 0;
        right_adm_recenter_count = 0;
        left_adm_recenter_cooldown = 0;
        right_adm_recenter_cooldown = 0;

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
                Vector3d ikL, ikR;
                mapContactTargetsToIkTargets(seed_vec, pL, pR, ikL, ikR);
                dualarm.SolveIK_Position(model, data, l_ik_EE, r_ik_EE, ikL, ikR, seed_vec, q_k);
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
        // 1) 차렷에서 양팔을 먼저 제자리 근처에서 위로 들어 올린다.
        addCartesianSegment(start_L, armRaiseL, start_R, armRaiseR, PHASE_APPROACH, 0.06);

        // 2) 들어 올린 높이를 유지한 채 큐브 위쪽의 wide pregrasp로 이동한다.
        addCartesianSegment(armRaiseL, pregraspHighWideL, armRaiseR, pregraspHighWideR, PHASE_APPROACH, 0.08);

        // 3) 큐브 바로 위쪽 정렬 자세로 안쪽(y) 정렬한다. 높은 z에서만 움직여 pedestal과 간섭을 피한다.
        addCartesianSegment(pregraspHighWideL, pregraspHighAlignL,
                            pregraspHighWideR, pregraspHighAlignR, PHASE_APPROACH, 0.06);

        // 4) 큐브 위에서 수직 하강하며 첫 contact 위치로 진입한다.
        // objL/R은 이미 박스 표면 안쪽 침투까지를 목표로 하므로, 이 구간은 낮은 속도로 내려 접촉 충격을 줄인다.
        const double APPROACH_CONTACT_V_DES = 0.02;
        addCartesianSegment(pregraspHighAlignL, objL, pregraspHighAlignR, objR, PHASE_APPROACH, APPROACH_CONTACT_V_DES);

        // 5) 첫 접촉 후, lift 전에 짧은 추가 압착으로 손끝 판이 더 면접촉에 가까워지도록 만든다.
        const double FINAL_SQUEEZE_V_DES = 0.01;
        addCartesianSegment(objL, squeezeL, objR, squeezeR, PHASE_GRASP_TO_PLACE, FINAL_SQUEEZE_V_DES);
        {
            int rows_before_lift = 0;
            for (const auto& seg : pos_segs) rows_before_lift += seg.rows();
            grasp_gate_row = rows_before_lift;
        }

        // 6) 추가 압착 상태에서 수직으로 들어올리기. 이 구간부터 어드미턴스가 계속 켜진 상태다.
        addCartesianSegment(squeezeL, liftL, squeezeR, liftR, PHASE_GRASP_TO_PLACE, 0.010);

        // 7) 들어올린 높이를 유지한 채 목표 지점으로 이동 (계속 PHASE_GRASP_TO_PLACE, 스퀴즈 유지)
        addCartesianSegment(liftL, transportL, liftR, transportR, PHASE_GRASP_TO_PLACE, 0.035);
        {
            int rows_before_return = 0;
            for (const auto& seg : pos_segs) rows_before_return += seg.rows();
            grasp_gate_end_row = rows_before_return;
        }

        // 8) 목표 지점에서 양팔을 바깥으로 벌려 물체를 놓는다.
        addCartesianSegment(transportL, releaseL, transportR, releaseR, PHASE_RETURN, 0.02);

        // 9) 내려놓기 완료 -> 원래 차렷 위치로 복귀
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
        ros::spinOnce();

        dual_arm_jointp[0] = waist_jointp[0];
        dual_arm_jointp[1] = head_jointp[0];
        dual_arm_jointp[2] = head_jointp[1];
        for (int i = 0; i < ARM_DOF; i++) {
            dual_arm_jointp[i + 3] = left_arm_jointp[i];
            dual_arm_jointp[i + 8] = right_arm_jointp[i];
        }

        dual_arm_jointv[0] = waist_jointv[0];
        dual_arm_jointv[1] = head_jointv[0];
        dual_arm_jointv[2] = head_jointv[1];
        for (int i = 0; i < ARM_DOF; i++) {
            dual_arm_jointv[i + 3] = left_arm_jointv[i];
            dual_arm_jointv[i + 8] = right_arm_jointv[i];
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
        for (int i = 0; i < ARM_DOF; i++) {
            dual_arm_jointp_vec(i + 3) = left_arm_jointp[i];
            dual_arm_jointp_vec(i + 8) = right_arm_jointp[i];
        }

        dual_arm_jointv_vec(0) = waist_jointv[0];
        dual_arm_jointv_vec(1) = head_jointv[0];
        dual_arm_jointv_vec(2) = head_jointv[1];
        for (int i = 0; i < ARM_DOF; i++) {
            dual_arm_jointv_vec(i + 3) = left_arm_jointv[i];
            dual_arm_jointv_vec(i + 8) = right_arm_jointv[i];
        }

        for (int i = 0; i < DoF; i++) {
            dual_arm_jointv_lpf_vec(i) = dual_arm_jointv_lpf[i];
        }

        const bool all_joint_states_received =
            waist_state_received && head_state_received && left_arm_state_received && right_arm_state_received;

        if (startup_state != STARTUP_COMPLETE) {
            if (!all_joint_states_received) {
                // Gazebo does not publish joint states while physics is paused.
                // The spawn pose is the documented zero standing pose, so begin
                // publishing its hold torque before the first physics step.
                if (!startup_zero_state_assumed) {
                    ROS_INFO("Startup: no joint state while paused; holding the zero standing pose.");
                    startup_zero_state_assumed = true;
                }
            }

            if (startup_state == STARTUP_WAIT_FOR_STATE) {
                startup_hold_q[0] = waist_jointp[0];
                startup_hold_q[1] = head_jointp[0];
                startup_hold_q[2] = head_jointp[1];
                for (int i = 0; i < ARM_DOF; ++i) {
                    startup_hold_q[i + 3] = left_arm_jointp[i];
                    startup_hold_q[i + 8] = right_arm_jointp[i];
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
                buildFixedDurationJointTrajectory(startup_hold_q, startup_target_q, STARTUP_MOVE_DURATION);
                traj_cnt = 0;
                traj_done_published = false;
                startup_state = STARTUP_MOVE_TO_TARGET;
                ROS_INFO("Startup: moving to standing pose over %.1f s.", STARTUP_MOVE_DURATION);
            }
            else if (startup_state == STARTUP_MOVE_TO_TARGET && traj_cnt >= dual_arm_jointp_trajectory.rows()) {
                startup_state = STARTUP_COMPLETE;
                ROS_INFO("Startup: standing pose reached, enabling normal command processing.");
            }
        }
        VectorXd nominal_joint_acc_vec = VectorXd::Zero(DoF);

        if(startup_state == STARTUP_COMPLETE && callback == true){
            bool new_trajectory_built = true;  // command_mode==3이 실패하면 false로 바뀌어 기존 궤적 재생을 유지

            // 현재 관절각을 initp(궤적 시작점) 및 IK 시드로 저장
            dual_arm_initp[0] = waist_jointp[0];
            dual_arm_initp[1] = head_jointp[0];
            dual_arm_initp[2] = head_jointp[1];
            for (int i = 0; i < ARM_DOF; i++) {
                dual_arm_initp[i + 3] = left_arm_jointp[i];
                dual_arm_initp[i + 8] = right_arm_jointp[i];
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

                Vector3d ikL, ikR;
                mapContactTargetsToIkTargets(q_ik_seed, tL, tR, ikL, ikR);
                dualarm.SolveIK_Position(model, data, l_ik_EE, r_ik_EE, ikL, ikR, q_ik_seed, q_ik_result);

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
                Vector3d startL = data.oMf[l_contact_EE].translation();
                Vector3d startR = data.oMf[r_contact_EE].translation();

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
                    Vector3d ikL, ikR;
                    mapContactTargetsToIkTargets(seed, pL, pR, ikL, ikR);
                    dualarm.SolveIK_Position(model, data, l_ik_EE, r_ik_EE, ikL, ikR, seed, q_k);

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
            bool grasp_gate_holding = false;
            if (grasp_gate_row > 0 &&
                traj_cnt >= grasp_gate_row &&
                (grasp_gate_end_row < 0 || traj_cnt < grasp_gate_end_row)) {
                if (!grasp_acquired_once) {
                    traj_cnt = grasp_gate_row - 1;  // 마지막 squeeze row에 고정, bilateral contact 전에는 lift 금지
                    grasp_gate_holding = true;
                } else if (!grasp_contact_ready) {
                    // 이동 중 파지력이 약해졌을 때 squeeze 구간으로 되감지 않는다.
                    // 현재 명목 자세를 유지한 채 arm-only admittance가 10N을
                    // 복구하도록 기다린 뒤, 같은 trajectory row부터 재개한다.
                    grasp_gate_holding = true;
                } else if (grasp_post_contact_hold_ticks < GRASP_POST_CONTACT_HOLD_TICKS) {
                    traj_cnt = grasp_gate_row - 1;  // 접촉 직후 그대로 더 조여서 면접촉을 안정화
                    grasp_post_contact_hold_ticks++;
                    grasp_gate_holding = true;
                }
            }
            for (int i = 0; i < DoF; i++){
                dual_arm_targetp[i] = dual_arm_jointp_trajectory(traj_cnt, i);
            }
            for (int i = 0; i < DoF; i++){
                dual_arm_targetv[i] = grasp_gate_holding ? 0.0 : dual_arm_jointv_trajectory(traj_cnt, i);
            }
            for (int i = 0; i < DoF; i++){
                nominal_joint_acc_vec(i) = grasp_gate_holding ? 0.0 : dual_arm_jointa_trajectory(traj_cnt, i);
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

            if (!grasp_gate_holding) {
                traj_cnt++;
            }
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
            bool grasp_just_acquired = false;
            // 실제 관절 상태에서의 FK/자코비안
            pinocchio::computeJointJacobians(model, data, dual_arm_jointp_vec);
            pinocchio::updateFramePlacements(model, data);

            Vector3d xL_actual = data.oMf[l_contact_EE].translation();
            Vector3d xR_actual = data.oMf[r_contact_EE].translation();
            Matrix3d RL_actual = data.oMf[l_contact_EE].rotation();
            Matrix3d RR_actual = data.oMf[r_contact_EE].rotation();

            pinocchio::Data::Matrix6x JL_full(6, model.nv); JL_full.setZero();
            pinocchio::Data::Matrix6x JR_full(6, model.nv); JR_full.setZero();
            pinocchio::getFrameJacobian(model, data, l_contact_EE, pinocchio::LOCAL_WORLD_ALIGNED, JL_full);
            pinocchio::getFrameJacobian(model, data, r_contact_EE, pinocchio::LOCAL_WORLD_ALIGNED, JR_full);
            MatrixXd JL = JL_full.topRows<3>();   // 위치 3행만 (DoF 열)
            MatrixXd JR = JR_full.topRows<3>();
            // Keep shoulder roll out of the unconstrained DLS solve. Including
            // it spreads the minimum-norm correction into roll, then the
            // posture clamp below discards that part and weakens both hands.
            constexpr int FORCE_CONTROL_DOF = 3;
            constexpr int L_FORCE_JOINT_IDX[FORCE_CONTROL_DOF] = {3, 5, 6};
            constexpr int R_FORCE_JOINT_IDX[FORCE_CONTROL_DOF] = {8, 10, 11};
            MatrixXd JL_arm(3, FORCE_CONTROL_DOF);
            MatrixXd JR_arm(3, FORCE_CONTROL_DOF);
            for (int c = 0; c < FORCE_CONTROL_DOF; ++c) {
                JL_arm.col(c) = JL.col(L_FORCE_JOINT_IDX[c]);
                JR_arm.col(c) = JR.col(R_FORCE_JOINT_IDX[c]);
            }

            // nominal target EE 위치 (target 궤적 기준 FK)
            pinocchio::forwardKinematics(model, data, dual_arm_targetp_vec);
            pinocchio::updateFramePlacements(model, data);
            const Vector3d xL_nominal = data.oMf[l_contact_EE].translation();
            const Vector3d xR_nominal = data.oMf[r_contact_EE].translation();
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
            // 각 패드의 local-Y 축을 실제 손바닥 법선으로 사용하되, 부호는
            // 항상 반대편 손을 향하도록 정규화한다. 따라서 허리/팔/손목이
            // 회전해도 힘 제어 방향이 world-Y에 고정되지 않는다.
            const Vector3d left_to_right = (xR_actual - xL_actual).normalized();
            Vector3d normal_L = RL_actual.col(1);
            Vector3d normal_R = RR_actual.col(1);
            if (normal_L.dot(left_to_right) < 0.0) normal_L = -normal_L;
            if (normal_R.dot(-left_to_right) < 0.0) normal_R = -normal_R;
            const double compressive_force_L = std::max(0.0, -F_ext_L.dot(normal_L));
            const double compressive_force_R = std::max(0.0, -F_ext_R.dot(normal_R));
            std_msgs::Float64 left_grasp_force_msg;
            std_msgs::Float64 right_grasp_force_msg;
            std_msgs::Float64 grasp_force_target_msg;
            left_grasp_force_msg.data = compressive_force_L;
            right_grasp_force_msg.data = compressive_force_R;
            // 파지 phase에서는 손목 정렬 여부와 무관하게 항상 10N을 명령한다.
            // 손목은 이 접촉력을 유지한 상태에서 수동적으로 면을 따라 정렬한다.
            grasp_force_target_msg.data = DESIRED_SQUEEZE_FORCE;
            left_grasp_force_pub.publish(left_grasp_force_msg);
            right_grasp_force_pub.publish(right_grasp_force_msg);
            grasp_force_target_pub.publish(grasp_force_target_msg);
            if (grasp_contact_ready) {
                if (compressive_force_L < GRASP_FORCE_LOSS_THRESHOLD ||
                    compressive_force_R < GRASP_FORCE_LOSS_THRESHOLD) {
                    grasp_force_loss_ticks++;
                    if (grasp_force_loss_ticks >= GRASP_FORCE_LOSS_TICKS) {
                        grasp_contact_ready = false;
                        grasp_contact_ticks = 0;
                        grasp_post_contact_hold_ticks = 0;
                        grasp_force_loss_ticks = 0;
                        // Once full face contact has been established, a brief
                        // force dip is a transport disturbance rather than a
                        // new alignment operation.  Preserve the compliant
                        // wrist alignment and let bilateral force recovery
                        // resume the held trajectory row.
                        wrist_alignment_ready = grasp_acquired_once;
                        wrist_alignment_ticks = 0;
                        ROS_WARN("Grasp force lost during motion; pausing nominal motion for admittance recovery (|Fy|=[%.2f %.2f] N)",
                                 compressive_force_L, compressive_force_R);
                    }
                } else {
                    grasp_force_loss_ticks = 0;
                }
            }
            if (!wrist_alignment_ready) {
                const bool alignment_contact =
                    compressive_force_L >= WRIST_ALIGN_CONTACT_FORCE &&
                    compressive_force_R >= WRIST_ALIGN_CONTACT_FORCE;
                const bool alignment_torque_ok =
                    std::abs(left_ft_torque_lpf(0)) <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                    std::abs(left_ft_torque_lpf(2)) <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                    std::abs(right_ft_torque_lpf(0)) <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                    std::abs(right_ft_torque_lpf(2)) <= WRIST_ALIGN_TORQUE_THRESHOLD;
                if (alignment_contact && alignment_torque_ok) {
                    wrist_alignment_ticks++;
                    if (wrist_alignment_ticks >= WRIST_ALIGN_HOLD_TICKS) {
                        wrist_alignment_ready = true;
                        ROS_INFO("Wrist face alignment ready (wrist remains compliant): qL=%.3f qR=%.3f rad, |Fy|=[%.2f %.2f] N",
                                 dual_arm_jointp[7], dual_arm_jointp[12],
                                 compressive_force_L, compressive_force_R);
                    }
                } else {
                    wrist_alignment_ticks = 0;
                }
            }
            if (wrist_alignment_ready && !grasp_contact_ready) {
                const bool left_contact_ok = compressive_force_L >= GRASP_CONTACT_FORCE_THRESHOLD;
                const bool right_contact_ok = compressive_force_R >= GRASP_CONTACT_FORCE_THRESHOLD;
                const bool left_face_ok = grasp_acquired_once ||
                    std::abs(left_ft_torque_lpf(0)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD &&
                    std::abs(left_ft_torque_lpf(2)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD;
                const bool right_face_ok = grasp_acquired_once ||
                    std::abs(right_ft_torque_lpf(0)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD &&
                    std::abs(right_ft_torque_lpf(2)) <= GRASP_FACE_CONTACT_TORQUE_THRESHOLD;
                if (left_contact_ok && right_contact_ok && left_face_ok && right_face_ok) {
                    grasp_contact_ticks++;
                    if (grasp_contact_ticks >= GRASP_CONTACT_HOLD_TICKS) {
                        const bool first_acquisition = !grasp_acquired_once;
                        grasp_contact_ready = true;
                        grasp_acquired_once = true;
                        grasp_just_acquired = first_acquisition;
                        ROS_INFO("Grasp face-contact ready: |Fy_L|=%.2f N |Fy_R|=%.2f N, |tau_xz_L|=[%.3f %.3f], |tau_xz_R|=[%.3f %.3f]",
                                 compressive_force_L, compressive_force_R,
                                 std::abs(left_ft_torque_lpf(0)), std::abs(left_ft_torque_lpf(2)),
                                 std::abs(right_ft_torque_lpf(0)), std::abs(right_ft_torque_lpf(2)));
                    }
                } else {
                    grasp_contact_ticks = 0;
                    grasp_post_contact_hold_ticks = 0;
                }
            }

            // 1차원 어드미턴스를 각 손바닥 법선 방향으로 적분한다.
            // left/right_adm_pos(1)는 world-Y 변위가 아니라 inward normal을
            // 따라간 스칼라 변위이며, 아래에서 world Cartesian 벡터로 변환한다.
            const double active_squeeze_force = DESIRED_SQUEEZE_FORCE;
            const double acc_L = (active_squeeze_force - compressive_force_L
                - Da_left[1] * left_adm_vel(1)) / Ma_left[1];
            const double acc_R = (active_squeeze_force - compressive_force_R
                - Da_right[1] * right_adm_vel(1)) / Ma_right[1];
            left_adm_vel(1) += acc_L * SAMPLING_TIME;
            right_adm_vel(1) += acc_R * SAMPLING_TIME;
            left_adm_vel(1) = std::max(-ADMITTANCE_VEL_LIMIT, std::min(ADMITTANCE_VEL_LIMIT, left_adm_vel(1)));
            right_adm_vel(1) = std::max(-ADMITTANCE_VEL_LIMIT, std::min(ADMITTANCE_VEL_LIMIT, right_adm_vel(1)));
            left_adm_pos(1) += left_adm_vel(1) * SAMPLING_TIME;
            right_adm_pos(1) += right_adm_vel(1) * SAMPLING_TIME;
            left_adm_pos(1) = std::max(-ADMITTANCE_POS_LIMIT, std::min(ADMITTANCE_POS_LIMIT, left_adm_pos(1)));
            right_adm_pos(1) = std::max(-ADMITTANCE_POS_LIMIT, std::min(ADMITTANCE_POS_LIMIT, right_adm_pos(1)));
            const Vector3d offset_L = normal_L * left_adm_pos(1);
            const Vector3d offset_R = normal_R * right_adm_pos(1);
            const Vector3d offset_vel_L = normal_L * left_adm_vel(1);
            const Vector3d offset_vel_R = normal_R * right_adm_vel(1);

            VectorXd dq_adm_L = dualarm.DampedPinv(JL_arm, ADMITTANCE_DLS_LAMBDA) * offset_L;
            VectorXd dq_adm_R = dualarm.DampedPinv(JR_arm, ADMITTANCE_DLS_LAMBDA) * offset_R;
            VectorXd dq_adm_dot_L = dualarm.DampedPinv(JL_arm, ADMITTANCE_DLS_LAMBDA) * offset_vel_L;
            VectorXd dq_adm_dot_R = dualarm.DampedPinv(JR_arm, ADMITTANCE_DLS_LAMBDA) * offset_vel_R;

            ROS_INFO_THROTTLE(
                1.0,
                "Squeeze diag: gap actual=%.3f nominal=%.3f m, admNormal L/R=[%.3f %.3f] m, "
                "Fnormal L/R=[%.2f %.2f] N, dq_norm L/R=[%.3f %.3f]",
                std::abs(xL_actual(1) - xR_actual(1)),
                std::abs(xL_nominal(1) - xR_nominal(1)),
                left_adm_pos(1), right_adm_pos(1),
                compressive_force_L, compressive_force_R, dq_adm_L.norm(), dq_adm_R.norm());

            for (int i = 0; i < FORCE_CONTROL_DOF; ++i) {
                dual_arm_targetp_vec(L_FORCE_JOINT_IDX[i]) += dq_adm_L(i);
                dual_arm_targetp_vec(R_FORCE_JOINT_IDX[i]) += dq_adm_R(i);
                dual_arm_targetv_vec(L_FORCE_JOINT_IDX[i]) += dq_adm_dot_L(i);
                dual_arm_targetv_vec(R_FORCE_JOINT_IDX[i]) += dq_adm_dot_R(i);
            }

            // The nominal IK applies these posture limits before admittance.
            // Reapply them to the final command so Cartesian squeeze cannot
            // fold the elbows inward or inherit a wrist-limit seed.
            constexpr double MAX_INWARD_SHOULDER_ROLL = 0.15;
            constexpr int L_SHOULDER_ROLL_IDX = 4;
            constexpr int L_WRIST_YAW_IDX = 7;
            constexpr int R_SHOULDER_ROLL_IDX = 9;
            constexpr int R_WRIST_YAW_IDX = 12;
            if (dual_arm_targetp_vec(L_SHOULDER_ROLL_IDX) < -MAX_INWARD_SHOULDER_ROLL) {
                dual_arm_targetp_vec(L_SHOULDER_ROLL_IDX) = -MAX_INWARD_SHOULDER_ROLL;
                dual_arm_targetv_vec(L_SHOULDER_ROLL_IDX) = 0.0;
            }
            if (dual_arm_targetp_vec(R_SHOULDER_ROLL_IDX) > MAX_INWARD_SHOULDER_ROLL) {
                dual_arm_targetp_vec(R_SHOULDER_ROLL_IDX) = MAX_INWARD_SHOULDER_ROLL;
                dual_arm_targetv_vec(R_SHOULDER_ROLL_IDX) = 0.0;
            }
            // Wrist yaw는 파지 중 각도를 잠그지 않는다. 매 tick 실제 각도를
            // nominal target으로 사용해 위치 스프링을 없애고 접촉면을 따라
            // 수동적으로 회전할 수 있게 한다.
            dual_arm_targetp_vec(L_WRIST_YAW_IDX) = dual_arm_jointp[L_WRIST_YAW_IDX];
            dual_arm_targetp_vec(R_WRIST_YAW_IDX) = dual_arm_jointp[R_WRIST_YAW_IDX];
            dual_arm_targetv_vec(L_WRIST_YAW_IDX) = 0.0;
            dual_arm_targetv_vec(R_WRIST_YAW_IDX) = 0.0;

            if (grasp_just_acquired &&
                traj_cnt >= 0 && traj_cnt < dual_arm_jointp_trajectory.rows()) {
                // Contact를 만드는 동안 사용한 compliance offset을 이후 이동용
                // 여유로 계속 들고 있으면 곧바로 +/-5cm limit에 포화된다.
                // 현재 보정된 arm target을 새 nominal grasp pose로 흡수하고,
                // 현재 파지 자세를 연속적으로 유지하면서 admittance travel을
                // 다시 확보하기 위해 이후 명목 궤적에 같은 joint offset을 적용한다.
                for (int i = 0; i < FORCE_CONTROL_DOF; ++i) {
                    const int joints[2] = {L_FORCE_JOINT_IDX[i], R_FORCE_JOINT_IDX[i]};
                    for (const int joint : joints) {
                        const double nominal_now = dual_arm_jointp_trajectory(traj_cnt, joint);
                        const double rebase_delta = dual_arm_targetp_vec(joint) - nominal_now;
                        dual_arm_jointp_trajectory.block(
                            traj_cnt, joint,
                            dual_arm_jointp_trajectory.rows() - traj_cnt, 1).array() += rebase_delta;
                    }
                }
                left_adm_pos.setZero();
                left_adm_vel.setZero();
                right_adm_pos.setZero();
                right_adm_vel.setZero();
                ROS_INFO("Admittance recentered at acquired grasp; full compliance travel restored for transport.");
            }

            // 이동 중 한쪽 팔이 compliance limit에 붙더라도 양팔의 현재
            // 보정 자세를 동시에 nominal trajectory에 흡수한다. 한 팔만
            // 옮기면 다음 row부터 상대 파지 자세가 바뀌어 반대 손의 힘이
            // 튀므로, 물체를 사이에 둔 bilateral equilibrium을 보존한다.
            if (left_adm_recenter_cooldown > 0) --left_adm_recenter_cooldown;
            if (right_adm_recenter_cooldown > 0) --right_adm_recenter_cooldown;
            // The initial acquisition rebase above is sufficient.  Rebasing
            // again during lift changes the future bilateral joint path and
            // was observed to tilt the box and stop the lift after ~1.7 cm.
            constexpr int MAX_ADM_RECENTERS_PER_ARM = 0;
            constexpr int ADM_RECENTER_COOLDOWN_TICKS = 2000;
            constexpr double ADM_RECENTER_TRIGGER = 0.045;
            constexpr double ADM_RECENTER_FORCE_CEILING = 9.2;
            auto rebaseArmTrajectory = [&](const int* joint_indices) {
                for (int i = 0; i < FORCE_CONTROL_DOF; ++i) {
                    const int joint = joint_indices[i];
                    const double nominal_now = dual_arm_jointp_trajectory(traj_cnt, joint);
                    const double rebase_delta = dual_arm_targetp_vec(joint) - nominal_now;
                    dual_arm_jointp_trajectory.block(
                        traj_cnt, joint,
                        dual_arm_jointp_trajectory.rows() - traj_cnt, 1).array() += rebase_delta;
                }
            };
            const bool left_recenter_needed =
                left_adm_pos(1) >= ADM_RECENTER_TRIGGER &&
                compressive_force_L < ADM_RECENTER_FORCE_CEILING;
            const bool right_recenter_needed =
                right_adm_pos(1) >= ADM_RECENTER_TRIGGER &&
                compressive_force_R < ADM_RECENTER_FORCE_CEILING;
            if (traj_cnt >= 0 && traj_cnt < dual_arm_jointp_trajectory.rows() &&
                grasp_acquired_once && !grasp_just_acquired &&
                left_adm_recenter_count < MAX_ADM_RECENTERS_PER_ARM &&
                right_adm_recenter_count < MAX_ADM_RECENTERS_PER_ARM &&
                left_adm_recenter_cooldown == 0 &&
                right_adm_recenter_cooldown == 0 &&
                (left_recenter_needed || right_recenter_needed)) {
                rebaseArmTrajectory(L_FORCE_JOINT_IDX);
                rebaseArmTrajectory(R_FORCE_JOINT_IDX);
                left_adm_pos.setZero();
                left_adm_vel.setZero();
                right_adm_pos.setZero();
                right_adm_vel.setZero();
                ++left_adm_recenter_count;
                ++right_adm_recenter_count;
                left_adm_recenter_cooldown = ADM_RECENTER_COOLDOWN_TICKS;
                right_adm_recenter_cooldown = ADM_RECENTER_COOLDOWN_TICKS;
                ROS_INFO("Bilateral admittance equilibrium recentered (%d/%d).",
                         left_adm_recenter_count, MAX_ADM_RECENTERS_PER_ARM);
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
            wrist_alignment_ready = false;
            wrist_alignment_ticks = 0;
            grasp_force_loss_ticks = 0;
            left_adm_recenter_count = 0;
            right_adm_recenter_count = 0;
            left_adm_recenter_cooldown = 0;
            right_adm_recenter_cooldown = 0;
            grasp_contact_ticks = 0;
            grasp_post_contact_hold_ticks = 0;
            if (task_phase != PHASE_GRASP_TO_PLACE) {
                grasp_contact_ready = false;
                grasp_acquired_once = false;
            }
        }

        // Keep the damping feedback unfiltered: a 10 Hz filter adds enough
        // phase lag to destabilize the stiff effort-controlled arm chain.
        dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
        for (int i = 0; i < DoF; i++) {
            dual_arm_targeta_vec(i) = nominal_joint_acc_vec(i) + PD_acc[i];
        }

        gravity_torque = pinocchio::computeGeneralizedGravity(model, data, dual_arm_jointp_vec);
        dynamic_torque = pinocchio::rnea(model, data, dual_arm_jointp_vec, dual_arm_jointv_lpf_vec, dual_arm_targeta_vec);
        for (int i = 0; i < DoF; i++) {
            target_torque[i] = dynamic_torque(i);
        }

        // Wrist inertia is too small for acceleration-level PD through RNEA
        // to produce a useful centering torque. Add a low-stiffness direct
        // spring so contact can still align the pads without hitting limits.
        constexpr double WRIST_CENTER_KP = 2.0;
        constexpr double WRIST_CENTER_KD = 0.15;
        constexpr int L_WRIST_YAW_IDX = 7;
        constexpr int R_WRIST_YAW_IDX = 12;
        // 접근 중에는 중앙으로 복귀시키되, 파지가 시작되면 위치 강성을
        // 완전히 제거해 손목이 접촉면을 따라 계속 순응하게 한다.
        const double wrist_target_l = admittance_active ? dual_arm_jointp[L_WRIST_YAW_IDX] : 0.0;
        const double wrist_target_r = admittance_active ? dual_arm_jointp[R_WRIST_YAW_IDX] : 0.0;
        const double wrist_kp = admittance_active ? 0.0 : WRIST_CENTER_KP;
        target_torque[L_WRIST_YAW_IDX] +=
            wrist_kp * (wrist_target_l - dual_arm_jointp[L_WRIST_YAW_IDX])
            -WRIST_CENTER_KD * dual_arm_jointv[L_WRIST_YAW_IDX];
        target_torque[R_WRIST_YAW_IDX] +=
            wrist_kp * (wrist_target_r - dual_arm_jointp[R_WRIST_YAW_IDX])
            -WRIST_CENTER_KD * dual_arm_jointv[R_WRIST_YAW_IDX];

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
            wrist_yaw_l_joint_msg.data      = dual_arm_targetp[7];
            shoulder_pitch_r_joint_msg.data = dual_arm_targetp[8];
            shoulder_roll_r_joint_msg.data  = dual_arm_targetp[9];
            shoulder_yaw_r_joint_msg.data   = dual_arm_targetp[10];
            elbow_r_joint_msg.data          = dual_arm_targetp[11];
            wrist_yaw_r_joint_msg.data      = dual_arm_targetp[12];
            head_yaw_joint_msg.data         = dual_arm_targetp[1];
            head_pitch_joint_msg.data       = dual_arm_targetp[2];

        // Effort Control
        #elif ARMCTRLMODE == EFFORT
            waist_joint_msg.data            = target_torque[0];
            head_yaw_joint_msg.data         = target_torque[1];
            head_pitch_joint_msg.data       = target_torque[2];
            shoulder_pitch_l_joint_msg.data = target_torque[3];
            shoulder_roll_l_joint_msg.data  = target_torque[4];
            shoulder_yaw_l_joint_msg.data   = target_torque[5];
            elbow_l_joint_msg.data          = target_torque[6];
            wrist_yaw_l_joint_msg.data      = target_torque[7];
            shoulder_pitch_r_joint_msg.data = target_torque[8];
            shoulder_roll_r_joint_msg.data  = target_torque[9];
            shoulder_yaw_r_joint_msg.data   = target_torque[10];
            elbow_r_joint_msg.data          = target_torque[11];
            wrist_yaw_r_joint_msg.data      = target_torque[12];

        #endif

        // ROS 토픽으로 조인트 명령 발행
        dual_armjoint1_pub.publish(waist_joint_msg);
        dual_armjoint2_pub.publish(shoulder_pitch_l_joint_msg);
        dual_armjoint3_pub.publish(shoulder_roll_l_joint_msg);
        dual_armjoint4_pub.publish(shoulder_yaw_l_joint_msg);
        dual_armjoint5_pub.publish(elbow_l_joint_msg);
        dual_armjoint6_pub.publish(wrist_yaw_l_joint_msg);
        dual_armjoint7_pub.publish(shoulder_pitch_r_joint_msg);
        dual_armjoint8_pub.publish(shoulder_roll_r_joint_msg);
        dual_armjoint9_pub.publish(shoulder_yaw_r_joint_msg);
        dual_armjoint10_pub.publish(elbow_r_joint_msg);
        dual_armjoint11_pub.publish(wrist_yaw_r_joint_msg);
        dual_armjoint12_pub.publish(head_yaw_joint_msg);
        dual_armjoint13_pub.publish(head_pitch_joint_msg);

        
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
    }
    
    return 0;
}
