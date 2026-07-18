#include "dual_arm_function.cpp"
#include <tf/transform_listener.h>
#include <vector>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <ros/package.h>

// place 목표 지점(이송 목표)을 눈으로 확인할 수 있도록 vision pick 명령이 들어올 때마다 스폰하는
// 표시용 모델. static이라 물리엔진과 무관 - 분홍색 구는 명령으로 입력한 좌표(dual_arm_commandx)
// 그 자체를 정확히 표시한다. 받침대(초록 박스)는 world 파일의 transport_fan_table 상판이 그
// 역할을 대신하게 되어 2026-07-18 제거함(사용자 요청).
// 매번 같은 이름("place_indicator")으로 스폰하므로, 재사용 전에 항상 delete부터 해서 이전 실행의
// 잔여물이 안 남게 한다.
const string PLACE_INDICATOR_SDF = R"(
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="place_indicator">
    <static>true</static>
    <link name="link">
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

void msgCallbackArucoPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_pose_cam = *msg;
    aruco_pose_received = true;
    aruco_pose_seq++;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Head 마커 탐색 (PHASE_SCAN)
// command_mode==3(vision pick) 진입 시 Head를 스캔 자세로 이동시키고 ArUco 검출을 기다린다.
// P0 진단 결과: Head_pitch는 +방향이 아래(물체 쪽)를 향한다 (좌표계 직관과 반대).
// 정지 후 0.5s 대기 -> 그 이후 도착하는 /aruco_ros/pose를 연속 3프레임 비교해서 서로 1cm 이내로
// 일치하면 검출 확정. 확인 시간이 타임아웃을 넘으면 ROS_WARN을 띄우고 실패 처리한다.
//
// 2026-07-18 (테이블/물체 z 변경 대응): 물체 높이가 바뀔 때마다 고정 pitch 한 값을 다시 맞춰야
// 했던 문제를 없애기 위해, 단일 고정 pitch 대신 위(pitch 작음, 덜 내려다봄)에서 아래(pitch 큼,
// 더 내려다봄)로 HEAD_SCAN_PITCH_STEPS 단계를 순서대로 훑는 방식으로 변경. 각 단계에서 기존과
// 동일한 SETTLE+CHECK(0.5s 정지 후 최대 4s, 3프레임 일치) 절차를 그대로 적용하고, 그 안에서
// 못 찾으면 다음 pitch로 넘어간다 - 어느 단계에서든 찾으면 그 즉시 확정하고 파지 파이프라인을
// 실행한다(끝까지 훑고도 못 찾을 때만 실패 처리).
//
// 2026-07-18 (좌우 추가): pitch 스윕만으로는 물체가 x,y로도 밀렸을 때(카메라 좌우 시야를 벗어남)
// 못 찾으므로, yaw도 HEAD_SCAN_YAW_STEPS 단계(좌->중앙->우)로 같이 훑는 2차원 그리드로 확장.
// pitch를 바깥 루프(위->아래), yaw를 안쪽 루프(좌->중앙->우)로 둬서 각 pitch 단계마다 좌우를
// 먼저 다 훑고 다음 pitch로 내려간다. 총 단계 수 = PITCH_STEPS * YAW_STEPS.
enum ScanStep { SCAN_MOVE, SCAN_SETTLE, SCAN_CHECK };

bool scan_active = false;
int scan_step = SCAN_MOVE;
int scan_wait_cnt = 0;
int scan_grid_idx = 0;                 // 현재 스윕 단계 인덱스 (0 ~ HEAD_SCAN_TOTAL_STEPS-1, pitch가 바깥/yaw가 안쪽)
unsigned long scan_last_seq = 0;
std::vector<Vector3d> scan_match_buf;  // 연속 프레임 일치 판정용 버퍼 (카메라 프레임 좌표)
double scan_q[DoF] = {0,};             // 스캔 진행 중 "현재 명령 관절각" (head만 갱신, 나머지는 스캔 시작 시점 값 유지)

// 2026-07-18: 실측(GUI 실행) 결과 마커가 스윕 범위의 맨 끝(yaw=-17.2deg, 3단계 중 좌측 끝)에서야
// 겨우 검출됨 - 실제 필요한 각도가 범위 경계에 아슬아슬하게 걸쳐 있어서, 조금만 더 벗어나면 15단계를
// 전부 훑고도 못 찾을 위험이 있었다(실측으로 실제 재현됨). ±0.3rad(17.2deg)에서 ±0.6rad(34.4deg)로
// 넉넉하게 확대.
// 주의(2차 실측으로 발견/수정): STEPS를 3으로 그대로 두고 범위만 넓히면 샘플 지점이
// {-34.4,0,34.4}deg로 바뀌어서, 기존에 실제로 검출됐던 -17.2deg 지점 자체가 더 이상 샘플에
// 포함되지 않는다(재현 확인: 새 범위로 재실행했더니 -34.4deg에서 못 찾고 멈춰 있었음). 범위를
// 넓히면서도 기존 성공 지점(17.2deg 간격)을 계속 포함하도록 STEPS를 3->5로 늘려
// {-34.4,-17.2,0,17.2,34.4}deg 5개 샘플이 되게 함.
const double HEAD_SCAN_YAW_LEFT       = -0.6;    // 스윕 좌측 끝 yaw [rad] (약 -34.4deg)
const double HEAD_SCAN_YAW_RIGHT      = 0.6;     // 스윕 우측 끝 yaw [rad] (약 +34.4deg)
const int    HEAD_SCAN_YAW_STEPS      = 5;       // -34.4/-17.2/0/17.2/34.4deg 5단계 (기존 실증된 -17.2deg 지점을 유지하면서 범위 확장)
const double HEAD_SCAN_PITCH_TOP      = 0.15;    // 스윕 시작 pitch(위쪽, 덜 내려다봄) [rad]
const double HEAD_SCAN_PITCH_BOTTOM   = 0.75;    // 스윕 종료 pitch(아래쪽, 더 내려다봄) [rad]
                                                  // 기존 단일 고정값(0.5236rad=30deg)이 이 범위 중앙 부근에 오도록 설정
const int    HEAD_SCAN_PITCH_STEPS    = 5;       // 스윕 단계 수(양끝 포함) - 0.15,0.30,0.45,0.60,0.75rad
const int    HEAD_SCAN_TOTAL_STEPS    = HEAD_SCAN_PITCH_STEPS * HEAD_SCAN_YAW_STEPS;  // 총 15단계
const int    SCAN_SETTLE_TICKS        = 500;     // 0.5s @ 1000Hz - 정지 후 카메라/인식 안정화 대기
const int    SCAN_CHECK_TIMEOUT_TICKS = 4000;    // 4s - 이 안에 3프레임 일치를 못 찾으면 다음 단계로
                                                  // (실측: aruco_ros 인식 속도가 ~1.5~2Hz에 간헐적으로 최대 ~1s 갭이 있음)
const int    SCAN_MATCH_FRAMES        = 3;       // 연속 일치 판정에 필요한 프레임 수
const double SCAN_MATCH_TOL           = 0.01;    // [m] 연속 프레임 간 허용 오차

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 콜백 함수들 (현재 조인트 상태 업데이트용)
// /dual_arm/joint_states 의 name 배열은 조인트 이름 알파벳순으로 정렬되어 온다(실측 확인).
// wrist-compliance-port 재작업(box-lifting 이식) Task 1에서 L_wrist_pitch_joint /
// R_wrist_pitch_joint 2개가 추가로 삽입되면서(urdf 조인트명 알파벳 정렬을 다시 확인,
// python xml.etree로 재검증) 알파벳순 인덱스가 또 한 번 밀렸다:
// Head_pitch_joint(0) < Head_yaw_joint(1) < L_elbow_joint(2) < L_shoulder_pitch_joint(3) <
// L_shoulder_roll_joint(4) < L_shoulder_yaw_joint(5) < L_wrist_pitch_joint(6) <
// L_wrist_yaw_joint(7) < R_elbow_joint(8) < R_shoulder_pitch_joint(9) <
// R_shoulder_roll_joint(10) < R_shoulder_yaw_joint(11) < R_wrist_pitch_joint(12) <
// R_wrist_yaw_joint(13) < Waist_joint(14)
void msgCallbackWaistArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        waist_jointp[0] = msg->position[14];
        waist_jointv[0] = msg->velocity[14];
        waist_torque[0] = msg->effort[14];
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

        // L_wrist_pitch_joint(알파벳순 인덱스 6, 신규) / L_wrist_yaw_joint(인덱스 7, L_shoulder_* 뒤)
        left_wrist_jointp[1] = msg->position[6];   // pitch
        left_wrist_jointv[1] = msg->velocity[6];
        left_wrist_jointp[0] = msg->position[7];   // yaw
        left_wrist_jointv[0] = msg->velocity[7];
    }

void msgCallbackRightArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        right_arm_jointp[0] = msg->position[9];
        right_arm_jointv[0] = msg->velocity[9];
        right_arm_torque[0] = msg->effort[9];

        right_arm_jointp[1] = msg->position[10];
        right_arm_jointv[1] = msg->velocity[10];
        right_arm_torque[1] = msg->effort[10];

        right_arm_jointp[2] = msg->position[11];
        right_arm_jointv[2] = msg->velocity[11];
        right_arm_torque[2] = msg->effort[11];

        right_arm_jointp[3] = msg->position[8];
        right_arm_jointv[3] = msg->velocity[8];
        right_arm_torque[3] = msg->effort[8];

        // R_wrist_pitch_joint(알파벳순 인덱스 12, 신규) / R_wrist_yaw_joint(인덱스 13, R_shoulder_* 뒤)
        right_wrist_jointp[1] = msg->position[12];   // pitch
        right_wrist_jointv[1] = msg->velocity[12];
        right_wrist_jointp[0] = msg->position[13];   // yaw
        right_wrist_jointv[0] = msg->velocity[13];
    }

// F/T 센서 콜백 (임피던스 제어의 F_ext로 사용)
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
        ros::Publisher dual_armjoint12_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint12_position_controller/command", 100);
        ros::Publisher dual_armjoint13_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint13_position_controller/command", 100);
        ros::Publisher dual_armjoint14_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint14_position_controller/command", 100);
        ros::Publisher dual_armjoint15_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint15_position_controller/command", 100);

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
        ros::Publisher dual_armjoint14_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint14_effort_controller/command", 100);
        ros::Publisher dual_armjoint15_pub = nh.advertise<std_msgs::Float64>("/dual_arm/joint15_effort_controller/command", 100);

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
    ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

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
    std_msgs::Float64 wrist_yaw_l_joint_msg, wrist_yaw_r_joint_msg;
    std_msgs::Float64 wrist_pitch_l_joint_msg, wrist_pitch_r_joint_msg;

    // 워크스페이스 위치에 무관하게 동작하도록 ROS 패키지 경로로 URDF를 찾는다
    string urdf_filename = ros::package::getPath("dual_arm") + "/urdf/dual_arm.urdf";
    // Pinocchio 모델 로드
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);

    // ===== 추가: EE 프레임 ID는 불변이므로 루프 밖에서 한 번만 구함 =====
    pinocchio::FrameIndex l_EE = model.getFrameId("L_EE_joint");
    pinocchio::FrameIndex r_EE = model.getFrameId("R_EE_joint");
    // wrist-compliance-port 재작업(box-lifting 이식, Task 1): pose-independent 원판 마운트 덕에
    // IK 타겟 프레임이 L_EE_joint/R_EE_joint와 기하학적으로 거의 일치하게 되어, 별도 손목 IK
    // 프레임 + 오프셋 리매핑(직전 계획 Task 1/4의 l_ik_EE/r_ik_EE/mapNominalTargetsToIkTargets)이
    // 불필요해져 제거함 - IK는 다시 L_EE_joint/R_EE_joint를 직접 타겟한다.
    // 접촉/어드미턴스 스퀴즈축 동적 계산용 grip 프레임 (Task 1에서 원판 중심에 추가, Task 4 Step 3에서 사용)
    pinocchio::FrameIndex l_grip_EE = model.getFrameId("L_grip_frame_joint");
    pinocchio::FrameIndex r_grip_EE = model.getFrameId("R_grip_frame_joint");

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

    // ===== 스윕 그리드 인덱스(0~HEAD_SCAN_TOTAL_STEPS-1) -> (yaw,pitch) [rad] 선형 보간 =====
    // pitch = idx / YAW_STEPS (바깥 루프, 위->아래), yaw = idx % YAW_STEPS (안쪽 루프, 좌->중앙->우)
    auto headScanYawForStep = [&](int idx) {
        int yaw_idx = idx % HEAD_SCAN_YAW_STEPS;
        if (HEAD_SCAN_YAW_STEPS <= 1) return HEAD_SCAN_YAW_LEFT;
        double t = (double)yaw_idx / (double)(HEAD_SCAN_YAW_STEPS - 1);
        return HEAD_SCAN_YAW_LEFT + t * (HEAD_SCAN_YAW_RIGHT - HEAD_SCAN_YAW_LEFT);
    };
    auto headScanPitchForStep = [&](int idx) {
        int pitch_idx = idx / HEAD_SCAN_YAW_STEPS;
        if (HEAD_SCAN_PITCH_STEPS <= 1) return HEAD_SCAN_PITCH_TOP;
        double t = (double)pitch_idx / (double)(HEAD_SCAN_PITCH_STEPS - 1);
        return HEAD_SCAN_PITCH_TOP + t * (HEAD_SCAN_PITCH_BOTTOM - HEAD_SCAN_PITCH_TOP);
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
        // 2026-07-18: 마커를 박스 로컬 -X 측면에서 +Z 윗면으로 옮김(models/aruco_box_26/model.sdf,
        // pose z=0.0755, 박스 15cm 절반+마커두께) - 보정 방향도 그에 맞춰 X축에서 Z축으로 변경.
        // 박스 중심 = 마커 위치 - (0, 0, 0.0755) (마커가 박스 중심보다 위에 있으므로 내려서 보정).
        // 윗면 오프셋은 박스가 yaw로 돌아가 있어도 항상 world -Z 방향 그대로 유지된다(요 회전축과
        // 오프셋 축이 같은 Z라 회전에 영향을 안 받음) - 옛 -X 측면 오프셋은 박스가 yaw로 돌면 world
        // 프레임에서 방향이 같이 돌아야 했는데 고정 오프셋이라 그 경우엔 원래도 부정확했던 문제가
        // 이번 변경으로 자연히 해소됨.
        // (주의: 처음에는 pose의 orientation(Z축=마커 법선)으로 회전에 무관하게 일반화해서 보정하려 했으나,
        //  이 시야각(오블리크)에서는 ArUco의 orientation 추정 자체가 부정확해서 오히려 오차가 커짐을 실측으로
        //  확인함. position(위치) 추정은 안정적이므로, 이 데모 world처럼 물체가 항상 축정렬로 스폰되는
        //  경우엔 world-frame 고정 오프셋이 orientation 기반 보정보다 더 안정적이다.)
        // (이 보정 없이 마커 위치를 그대로 물체 중심으로 쓰면 grasp_offset 스퀴즈가 실제 박스 표면을
        //  몇 cm씩 빗나가 파지가 전혀 안 되는 문제가 있었음 - 실측으로 확인.)
        const double MARKER_TO_BOX_CENTER = 0.0755;

        Vector3d obj = Vector3d(object_world.pose.position.x,
                                 object_world.pose.position.y,
                                 object_world.pose.position.z)
                       + Vector3d(0, 0, -MARKER_TO_BOX_CENTER);
        Vector3d transport_pt(dual_arm_commandx[0], dual_arm_commandx[1], dual_arm_commandx[2]);

        // 오라클(시뮬레이션 전용): 실제 인식 파이프라인이 아니라 Gazebo ground truth로 물체의
        // 실제 yaw를 직접 읽는다. ArUco pose는 위치만 쓰고(위 MARKER_TO_BOX_CENTER 보정 참고,
        // 기존 코드가 방향 추정은 오블리크 각도에서 부정확하다고 판단해 안 씀), 물체가 회전된
        // 상태에서 접근 기하 자체를 맞추려면 최소한 yaw는 알아야 한다. 실제 로봇에서는 이 블록을
        // 깊이 카메라 기반 평면 피팅 등 진짜 인식으로 교체해야 한다 - 지금은 "접근 기하를
        // 회전시키면 모서리 충돌이 없어지는지"를 검증하기 위한 자리표시자.
        double obj_yaw = 0.0;
        {
            gazebo_msgs::GetModelState state_srv;
            state_srv.request.model_name = "aruco_box_26";
            if (get_model_state_client.call(state_srv) && state_srv.response.success) {
                const auto& q = state_srv.response.pose.orientation;
                obj_yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
            } else {
                ROS_WARN("GetModelState(aruco_box_26) failed - assuming yaw=0");
            }
        }
        const Eigen::Matrix3d obj_yaw_rot = Eigen::AngleAxisd(obj_yaw, Vector3d::UnitZ()).toRotationMatrix();

        // 양팔 동시 파지 간격(물체를 y축 양쪽에서 감싸는 형태)
        // aruco_box_26 기준: 15cm 정육면체, y방향 half-width = 0.075m
        // grasp_offset = half-width - 침투깊이(10mm, 기존 튜닝값 그대로 유지) = 0.075 - 0.010 = 0.065
        const double grasp_offset = 0.065;  // 물체/이송목표 좌우 간격 (표면 안쪽 10mm 압착 - 기존 5mm는 정적 유지 여유만 있고
                                             // 이송 중 관성부하를 버틸 마진이 없어 슬립 발생, Kd_imp 상향과 함께 조임)

        VectorXd base_seed(DoF);
        for (int i = 0; i < DoF; i++) base_seed(i) = base_q[i];

        pinocchio::forwardKinematics(model, data, base_seed);
        pinocchio::updateFramePlacements(model, data);
        Vector3d start_L = data.oMf[l_EE].translation();
        Vector3d start_R = data.oMf[r_EE].translation();

        // 물체/이송목표 좌우 접근점 (물체의 실제 yaw만큼 회전된 축 양쪽에서 감싸는 자세).
        // squeeze_dir는 "중심 -> 왼손" 방향의 단위벡터를 물체 yaw로 회전시킨 것 - 이 방향을
        // objL/R뿐 아니라 transportL/R에도 동일하게 적용해서, 파지~이송 내내(스퀴즈를 쥐고 있는
        // 동안) 양손 간격 방향이 도중에 바뀌지 않도록 한다(안 그러면 lift/transport 전환 시점에
        // 손 간격 방향이 갑자기 world-Y로 스냅해 쥐고 있던 그립에 충격을 준다).
        const Vector3d squeeze_dir = obj_yaw_rot * Vector3d(0, grasp_offset, 0);
        Vector3d objL = obj + squeeze_dir;
        Vector3d objR = obj - squeeze_dir;
        Vector3d transportL = transport_pt + squeeze_dir;
        Vector3d transportR = transport_pt - squeeze_dir;

        // pick_pedestal(world 파일)이 파지점 바로 아래(z 1.05~1.15)에 y로 걸쳐 있어서, 시작 자세에서
        // objL/R로 곧장 3D 직선 이동하면 z가 받침대 상판보다 낮은 구간에서 x,y가 이미 받침대 영역에
        // 들어가 팔이 모서리에 부딪힌다. 그래서 접근을 2단계로 나눈다: 먼저 파지 높이(obj.z, 받침대
        // 상판보다 7.5cm 위)를 유지한 채 받침대 바깥쪽으로 STANDOFF_Y만큼 더 벌어진 standoff 지점으로
        // 이동하고, 그다음 그 높이를 유지한 채 (물체가 yaw만큼 돌아가 있으면 그 축을 따라) 직선
        // 이동해 파지점에 들어간다 - 패드가 물체 면에 거의 수직으로 들어가 모서리를 안 침.
        const double STANDOFF_Y = 0.15;  // 받침대 y 반폭(0.085)보다 충분히 큰 여유
        const Vector3d standoff_dir = obj_yaw_rot * Vector3d(0, STANDOFF_Y, 0);
        Vector3d standoffL = objL + standoff_dir;
        Vector3d standoffR = objR - standoff_dir;

        // 2026-07-18: transport_fan_table이 옛 받침대(17x17cm)보다 훨씬 큰 단일 상판(0.6x1.4m)으로
        // 바뀌면서, 대기 자세(start_L/R)에서 standoffL/R로 곧장 가는 대각선 직선이 테이블 상판
        // 위를 스치듯 지나가 걸리는 문제가 발생함(STANDOFF_Y는 옛 받침대 기준 클리어런스라 지금
        // 넓은 테이블에는 부족). 그래서 이 구간을 "제자리에서 안전 높이로 상승 -> standoff로 이동"
        // 2단계로 나눈다(사용자 확인: 안전 높이에 오른 뒤부터는 그대로 standoff까지 한 번에 가도
        // 됨 - 어차피 그 사이 이동은 y 위주라 대각선 하강 도중 테이블에 걸릴 일이 없음). SAFE_TRANSIT_Z는
        // 테이블 상판(z=1.0)보다 5cm 위로 잡아 상승 시점에는 상판을 확실히 넘어가게 함.
        // 2026-07-18 (자세 붕괴 방지): 실측(GUI 실행) 결과 이 상승 구간이 x,y 변화 없이 z만 바뀌는
        // 순수 수직 직선이라, 대기 자세에서 팔을 곧게 편 채로만 올라가 어깨/팔꿈치가 부자연스럽게
        // 꺾이는 자세가 나옴. SAFE_TRANSIT_FORWARD_X만큼 로봇 정면(+X, world 기준) 쪽으로도 같이
        // 이동시켜 순수 수직 대신 완만한 대각선 궤적이 되도록 함 - 팔꿈치가 자연스럽게 앞으로
        // 굽으면서 올라가 특정 자세로 급격히 꺾이는 문제를 줄인다.
        const double SAFE_TRANSIT_Z = 1.05;  // 테이블 상판(z=1.0)보다 위 - 접근 전 안전 이동 높이
        const double SAFE_TRANSIT_FORWARD_X = 0.15;  // 상승과 함께 앞으로 살짝 이동 - 순수 수직 상승 시 자세 붕괴 방지
        Vector3d liftoffL(start_L.x() + SAFE_TRANSIT_FORWARD_X, start_L.y(), SAFE_TRANSIT_Z);
        Vector3d liftoffR(start_R.x() + SAFE_TRANSIT_FORWARD_X, start_R.y(), SAFE_TRANSIT_Z);

        // 파지 직후 곧바로 파지점->이송목표 대각선 직선으로 이동하면 받침대/바닥 근처를 스치듯 지나갈
        // 수 있다. 스퀴즈를 유지한 채(PHASE_GRASP_TO_PLACE) 먼저 수직으로 LIFT_HEIGHT만큼 들어올린 뒤,
        // 그 높이에서 이송목표로 이동한다.
        const double LIFT_HEIGHT = 0.10;  // 파지 높이에서 들어올릴 여유 [m]
        Vector3d liftL = objL + Vector3d(0, 0, LIFT_HEIGHT);
        Vector3d liftR = objR + Vector3d(0, 0, LIFT_HEIGHT);

        std::vector<MatrixXd> pos_segs, vel_segs, acc_segs;
        std::vector<MatrixXd> cart_pos_segs, cart_vel_segs, cart_acc_segs;  // 어드미턴스용 desired Cartesian(x_d,ẋ_d,ẍ_d)
        std::vector<int> seg_phase;   // 세그먼트별 TaskPhase 태그 (재생 중 자동 전환용)
        VectorXd seed_vec = base_seed;

        // Cartesian 직선 구간 하나를 만들어 세그먼트 목록에 추가.
        // CartesianLineTrajectory로 6D(L+R) 직선 경로를 만들고, 매 웨이포인트마다 DLS IK를 풀어
        // 관절각 시퀀스로 변환한 뒤, 위치->속도->가속도를 중심차분으로 계산한다 (mode 2 cartesian sim과 동일 방식).
        // 2026-07-18 (37초 프리징 해결): 실측(GUI 실행) 결과, 전체 파이프라인(약 7개 세그먼트,
        // 1000Hz 샘플링 기준 도합 수만 개 웨이포인트)에서 웨이포인트마다 DLS IK(최대 300회 반복)를
        // 매번 새로 푸느라 재생 시작 전에 30초 넘게 멈춰 있었다. dual_arm_function.cpp의
        // SolveIK_Position/CartesianLineTrajectory 자체(공용 유틸/다른 모드에서도 씀)는 손대지
        // 않고, 이 안(main.cpp의 웨이포인트->관절각 변환 루프)에서만 IK_SOLVE_STRIDE개 웨이포인트
        // 마다 한 번만 실제 IK를 풀고, 그 사이 구간은 선형보간으로 채운다 - 1000Hz 결과물(jp)의
        // 크기/이후 속도·가속도 중심차분 로직은 그대로 유지하면서 IK 호출 횟수만 1/stride로 줄인다.
        const int IK_SOLVE_STRIDE = 20;  // 20ms마다 한 번 IK, 나머지는 보간 (재생 시작 지연을 수십초 -> 1~2초로 단축)

        // 2026-07-18 (머리로 물체 추적): 파지 전/파지 후 이동 내내 pL,pR의 중점(=물체가 손에 들려
        // 있으면 정확히 물체 중심, 파지 전이면 목표 접근점 중심)을 바라보도록 head yaw를 매 웨이포인트
        // 재계산한다. Head_yaw_joint는 waist(Waist_joint) 기준 로컬 회전이라, world 기준 목표 방위각
        // (atan2)에서 그 순간의 waist 각도(q_k(0))를 빼야 실제로 world상 같은 지점을 계속 본다(waist가
        // 돌아가는 동안에도 머리가 목표에서 벗어나지 않음). pitch는 카메라 마운트 오프셋 보정 없이
        // 매 순간 재계산하면 오히려 부정확해질 수 있어, 스캔이 이미 검증한 값(scan_q[2])을 그대로 쓴다.
        // track_head=false로 부르면 정면(0,0)으로 고정 - 복귀 마지막 구간에 사용.
        auto headLookAt = [&](const Vector3d& pL, const Vector3d& pR, double waist_angle) {
            Vector3d target = 0.5 * (pL + pR);
            double world_bearing = std::atan2(target.y(), target.x());
            return world_bearing - waist_angle;
        };

        auto addCartesianSegment = [&](const Vector3d& sL, const Vector3d& gL,
                                        const Vector3d& sR, const Vector3d& gR, int phase,
                                        double v_des = 0.1, bool track_head = true) {
            MatrixXd cart_p, cart_v, cart_a;
            dualarm.CartesianLineTrajectory(sL, gL, sR, gR, cart_p, cart_v, cart_a, v_des);
            int steps = cart_p.rows();
            cart_pos_segs.push_back(cart_p);
            cart_vel_segs.push_back(cart_v);
            cart_acc_segs.push_back(cart_a);

            MatrixXd jp(steps, DoF), jv(steps, DoF), ja(steps, DoF);

            // 1) IK_SOLVE_STRIDE 간격으로만 실제 IK를 풀고(끝점은 항상 포함), 나머지 웨이포인트는
            //    양쪽 IK 결과 사이를 선형보간해서 채운다.
            std::vector<int> ik_idx;
            for (int k = 0; k < steps; k += IK_SOLVE_STRIDE) ik_idx.push_back(k);
            if (ik_idx.back() != steps - 1) ik_idx.push_back(steps - 1);

            MatrixXd jp_coarse((int)ik_idx.size(), DoF);
            for (size_t ci = 0; ci < ik_idx.size(); ci++) {
                int k = ik_idx[ci];
                Vector3d pL(cart_p(k,0), cart_p(k,1), cart_p(k,2));
                Vector3d pR(cart_p(k,3), cart_p(k,4), cart_p(k,5));
                VectorXd q_k;
                dualarm.SolveIK_Position(model, data, l_EE, r_EE, pL, pR, seed_vec, q_k);
                if (track_head) {
                    q_k(1) = headLookAt(pL, pR, q_k(0));  // head yaw: 물체 중점을 world 기준으로 계속 바라봄
                    q_k(2) = scan_q[2];                    // head pitch: 스캔이 확정한 값 유지
                } else {
                    q_k(1) = 0.0;  // 정면(yaw=0)
                    q_k(2) = 0.0;  // 정면(pitch=0)
                }
                for (int i = 0; i < DoF; i++) jp_coarse((int)ci, i) = q_k(i);
                seed_vec = q_k;   // 다음 웨이포인트/다음 세그먼트로 시드 연속성 유지
            }
            for (size_t ci = 0; ci + 1 < ik_idx.size(); ci++) {
                int k0 = ik_idx[ci], k1 = ik_idx[ci + 1];
                for (int k = k0; k < k1; k++) {
                    double alpha = (double)(k - k0) / (double)(k1 - k0);
                    for (int i = 0; i < DoF; i++)
                        jp(k, i) = (1.0 - alpha) * jp_coarse((int)ci, i) + alpha * jp_coarse((int)ci + 1, i);
                }
            }
            for (int i = 0; i < DoF; i++) jp(steps - 1, i) = jp_coarse((int)ik_idx.size() - 1, i);

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
        // 0) 대기 자세에서 제자리(x,y 그대로) 안전 높이(SAFE_TRANSIT_Z)까지 수직 상승
        addCartesianSegment(start_L, liftoffL, start_R, liftoffR, PHASE_APPROACH);

        // 0b) 안전 높이에서 standoff로 곧장 이동 (테이블 상판보다 위에서 출발하므로 대각선이어도 걸리지 않음)
        addCartesianSegment(liftoffL, standoffL, liftoffR, standoffR, PHASE_APPROACH);

        // 1b) standoff -> 파지 위치로 y 방향 직선 접근 (파지 높이를 그대로 유지하므로 받침대와 부딪히지 않음)
        // objL/R은 이미 박스 표면 안쪽(grasp_offset 침투)까지를 목표로 하므로, 기본 속도(0.1m/s)로
        // 그대로 들어가면 접촉 순간 충격이 커서 좌우 접촉이 어긋나며 물체가 회전하며 떨어지는 문제가
        // 있었다 - 이 구간만 더 느리게(v_des=0.03m/s) 접근해 접촉 충격을 줄인다.
        const double APPROACH_CONTACT_V_DES = 0.03;
        addCartesianSegment(standoffL, objL, standoffR, objR, PHASE_APPROACH, APPROACH_CONTACT_V_DES);

        // 2) 파지 위치에서 수직으로 들어올리기. 이 구간부터 PHASE_GRASP_TO_PLACE로 태깅되어 임피던스
        //    제어가 켜지고, 양팔 스퀴즈(grasp_offset) 마찰로 물체를 실제로 붙잡아 든다.
        addCartesianSegment(objL, liftL, objR, liftR, PHASE_GRASP_TO_PLACE);

        // 2b) 들어올린 높이를 유지한 채 목표 지점으로 이동 (계속 PHASE_GRASP_TO_PLACE, 스퀴즈 유지)
        addCartesianSegment(liftL, transportL, liftR, transportR, PHASE_GRASP_TO_PLACE);

        // 3) 놓기: transportL/R에 그대로 머물며(변위 0) 스퀴즈 목표힘을 램프다운 - 아래 온라인
        //    루프의 PHASE_RELEASE 분기가 이 dwell 구간(1000 tick, CartesianLineTrajectory가
        //    거리 0일 때 Tf=1.0으로 fallback하는 것을 그대로 이용) 동안 힘을 0으로 스르륵 뺀다.
        addCartesianSegment(transportL, transportL, transportR, transportR, PHASE_RELEASE);

        // 3b) 후퇴: 스퀴즈가 이미 0으로 빠진 상태에서, 파지 때와 같은 축(squeeze_dir)을 따라
        //     양손을 서로 반대 방향으로 RETREAT_DIST만큼 벌려 물체에서 확실히 손을 뗀다.
        const Vector3d retreat_dir = squeeze_dir.normalized() * RETREAT_DIST;
        Vector3d retreatL = transportL + retreat_dir;
        Vector3d retreatR = transportR - retreat_dir;
        addCartesianSegment(transportL, retreatL, transportR, retreatR, PHASE_RETURN);

        // 3c) 테이블 이탈: 후퇴 지점(테이블 위, z~1.075)에서 곧장 대기 자세로 대각선으로 가면
        // 갈 때와 대칭적으로 테이블 위를 스치듯 지나갈 위험이 있다(0)/0b)와 동일한 문제). 그래서
        // liftoffL/R(갈 때 썼던 안전 높이 경유점)을 재사용해 높이를 유지한 채 먼저 테이블을
        // 빠져나가고, 그다음(3d)에야 원래 자세로 내려간다.
        addCartesianSegment(retreatL, liftoffL, retreatR, liftoffR, PHASE_RETURN);

        // 3d) 복귀: 테이블을 벗어난 안전 지점에서 원래 대기 자세로. 여기서부터는 물체도 이미
        // 내려놓았고 카메라로 추적할 대상이 없으므로 머리를 정면(0,0)으로 되돌린다(track_head=false).
        addCartesianSegment(liftoffL, start_L, liftoffR, start_R, PHASE_RETURN, 0.1, false);

        ROS_INFO("Vision pick(dual-arm): object(world)=[%.3f %.3f %.3f], transport=[%.3f %.3f %.3f]",
                 obj.x(), obj.y(), obj.z(),
                 transport_pt.x(), transport_pt.y(), transport_pt.z());

        int total_rows = 0;
        for (auto& s : pos_segs) total_rows += s.rows();
        dual_arm_jointp_trajectory.resize(total_rows, DoF);
        dual_arm_jointv_trajectory.resize(total_rows, DoF);
        dual_arm_jointa_trajectory.resize(total_rows, DoF);
        dual_arm_phase_trajectory.resize(total_rows);
        dual_arm_cart_target_trajectory.resize(total_rows, 6);
        dual_arm_cart_target_vel_trajectory.resize(total_rows, 6);
        dual_arm_cart_target_acc_trajectory.resize(total_rows, 6);
        int offset = 0;
        for (size_t k = 0; k < pos_segs.size(); ++k) {
            int r = pos_segs[k].rows();
            dual_arm_jointp_trajectory.block(offset, 0, r, DoF) = pos_segs[k];
            dual_arm_jointv_trajectory.block(offset, 0, r, DoF) = vel_segs[k];
            dual_arm_jointa_trajectory.block(offset, 0, r, DoF) = acc_segs[k];
            dual_arm_phase_trajectory.segment(offset, r).setConstant(seg_phase[k]);
            dual_arm_cart_target_trajectory.block(offset, 0, r, 6)     = cart_pos_segs[k];
            dual_arm_cart_target_vel_trajectory.block(offset, 0, r, 6) = cart_vel_segs[k];
            dual_arm_cart_target_acc_trajectory.block(offset, 0, r, 6) = cart_acc_segs[k];
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
                ROS_INFO("Head scan: marker confirmed at sweep step %d/%d (yaw=%.1fdeg, pitch=%.1fdeg)",
                         scan_grid_idx + 1, HEAD_SCAN_TOTAL_STEPS, scan_q[1] * rad2deg, scan_q[2] * rad2deg);
                scan_active = false;
                if (!buildGraspPipelineFromDetection(scan_q)) {
                    task_phase = PHASE_APPROACH;   // TF 실패 -> 실패 처리, 접근 단계로 리셋
                }
                return;
            }

            scan_wait_cnt++;
            if (scan_wait_cnt >= SCAN_CHECK_TIMEOUT_TICKS) {
                if (scan_grid_idx + 1 < HEAD_SCAN_TOTAL_STEPS) {
                    // 이 단계에서 못 찾음 -> 다음 (yaw 먼저, 한 바퀴 돌면 pitch 한 단계 아래로) 조합으로 재시도
                    scan_grid_idx++;
                    ROS_INFO("Head scan: not found at step %d/%d (yaw=%.1fdeg, pitch=%.1fdeg), moving to next.",
                             scan_grid_idx, HEAD_SCAN_TOTAL_STEPS, scan_q[1] * rad2deg, scan_q[2] * rad2deg);
                    startScanMoveTo(headScanYawForStep(scan_grid_idx), headScanPitchForStep(scan_grid_idx));
                    return;
                }
                ROS_WARN("Head scan: object not found after sweeping all %d steps (yaw %.1fdeg~%.1fdeg x pitch %.1fdeg~%.1fdeg).",
                         HEAD_SCAN_TOTAL_STEPS, HEAD_SCAN_YAW_LEFT * rad2deg, HEAD_SCAN_YAW_RIGHT * rad2deg,
                         HEAD_SCAN_PITCH_TOP * rad2deg, HEAD_SCAN_PITCH_BOTTOM * rad2deg);
                scan_active = false;
                task_phase = PHASE_APPROACH;   // 실패 처리: 접근 단계로 리셋, 마지막 자세에서 정지 유지
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
            dual_arm_jointp[i + 9] = right_arm_jointp[i];
        }
        dual_arm_jointp[7]  = left_wrist_jointp[0];
        dual_arm_jointp[8]  = left_wrist_jointp[1];
        dual_arm_jointp[13] = right_wrist_jointp[0];
        dual_arm_jointp[14] = right_wrist_jointp[1];

        dual_arm_jointv[0] = waist_jointv[0];
        dual_arm_jointv[1] = head_jointv[0];
        dual_arm_jointv[2] = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv[i + 3] = left_arm_jointv[i];
            dual_arm_jointv[i + 9] = right_arm_jointv[i];
        }
        dual_arm_jointv[7]  = left_wrist_jointv[0];
        dual_arm_jointv[8]  = left_wrist_jointv[1];
        dual_arm_jointv[13] = right_wrist_jointv[0];
        dual_arm_jointv[14] = right_wrist_jointv[1];

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
            dual_arm_jointp_vec(i + 9) = right_arm_jointp[i];
        }
        dual_arm_jointp_vec(7)  = left_wrist_jointp[0];
        dual_arm_jointp_vec(8)  = left_wrist_jointp[1];
        dual_arm_jointp_vec(13) = right_wrist_jointp[0];
        dual_arm_jointp_vec(14) = right_wrist_jointp[1];

        dual_arm_jointv_vec(0) = waist_jointv[0];
        dual_arm_jointv_vec(1) = head_jointv[0];
        dual_arm_jointv_vec(2) = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_vec(i + 3) = left_arm_jointv[i];
            dual_arm_jointv_vec(i + 9) = right_arm_jointv[i];
        }
        dual_arm_jointv_vec(7)  = left_wrist_jointv[0];
        dual_arm_jointv_vec(8)  = left_wrist_jointv[1];
        dual_arm_jointv_vec(13) = right_wrist_jointv[0];
        dual_arm_jointv_vec(14) = right_wrist_jointv[1];

        dual_arm_jointv_lpf_vec(0) = waist_jointv[0];
        dual_arm_jointv_lpf_vec(1) = head_jointv[0];
        dual_arm_jointv_lpf_vec(2) = head_jointv[1];
        for (int i = 0; i < 4; i++) {
            dual_arm_jointv_lpf_vec(i + 3) = left_arm_jointv[i];
            dual_arm_jointv_lpf_vec(i + 9) = right_arm_jointv[i];
        }
        dual_arm_jointv_lpf_vec(7)  = left_wrist_jointv[0];
        dual_arm_jointv_lpf_vec(8)  = left_wrist_jointv[1];
        dual_arm_jointv_lpf_vec(13) = right_wrist_jointv[0];
        dual_arm_jointv_lpf_vec(14) = right_wrist_jointv[1];



        if(callback == true){
            bool new_trajectory_built = true;  // command_mode==3이 실패하면 false로 바뀌어 기존 궤적 재생을 유지

            // 현재 관절각을 initp(궤적 시작점) 및 IK 시드로 저장
            dual_arm_initp[0] = waist_jointp[0];
            dual_arm_initp[1] = head_jointp[0];
            dual_arm_initp[2] = head_jointp[1];
            for (int i = 0; i < 4; i++) {
                dual_arm_initp[i + 3] = left_arm_jointp[i];
                dual_arm_initp[i + 9] = right_arm_jointp[i];
            }
            dual_arm_initp[7]  = left_wrist_jointp[0];
            dual_arm_initp[8]  = left_wrist_jointp[1];
            dual_arm_initp[13] = right_wrist_jointp[0];
            dual_arm_initp[14] = right_wrist_jointp[1];
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

            // ===== 모드 3: vision pick (Head 이동 -> ArUco 검출 -> world 변환 -> 파지/이송/복귀) =====
            // 새 vision pick 명령이 들어올 때마다 항상 검출부터 새로 시작한다 - 이전에 우연히 잡혔을 수
            // 있는 오래된 aruco_pose_cam을 그대로 재사용하지 않기 위해 aruco_pose_received를 강제로 리셋한다.
            // (그 다음 실제 파지 파이프라인 생성은 검출을 확정한 뒤 runScanStep()에서 수행된다.)
            else if (command_mode == 3) {
                double tx = dual_arm_commandx[0], ty = dual_arm_commandx[1], tz = dual_arm_commandx[2];

                aruco_pose_received = false;
                scan_match_buf.clear();
                for (int i = 0; i < DoF; i++) scan_q[i] = dual_arm_initp[i];

                // 이번 명령의 이송 목표 위치에 표시/받침대(place_indicator)를 스폰 - 스캔/파지가
                // 끝나기 한참 전인 지금 미리 만들어둬야, 이후 이송 중에도 목표 지점을 눈으로 계속 볼 수 있다.
                spawnPlaceIndicator(Vector3d(tx, ty, tz));

                scan_grid_idx = 0;
                ROS_INFO("Vision pick: sweeping head yaw %.1fdeg~%.1fdeg x pitch %.1fdeg~%.1fdeg (%d steps total) to find marker.",
                         HEAD_SCAN_YAW_LEFT * rad2deg, HEAD_SCAN_YAW_RIGHT * rad2deg,
                         HEAD_SCAN_PITCH_TOP * rad2deg, HEAD_SCAN_PITCH_BOTTOM * rad2deg, HEAD_SCAN_TOTAL_STEPS);

                startScanMoveTo(headScanYawForStep(scan_grid_idx), headScanPitchForStep(scan_grid_idx));
                scan_active = true;
            }

            if (new_trajectory_built) {
                traj_cnt = 0;
                traj_done_published = false;   // 새 궤적 시작 -> 완료 알림 다시 대기
                wrist_alignment_ready = false;  // 새 파지마다 정렬 게이트 재무장
                wrist_alignment_ticks = 0;
            }
            callback = false;
        }
        else if (traj_cnt < dual_arm_jointp_trajectory.rows()){
            // vision pick(mode 3) 재생 중이면 현재 행에 태깅된 phase로 자동 전환.
            // (mode 0/1/2는 전 구간 PHASE_APPROACH로 태깅되어 있어 아래 어드미턴스 분기가 절대 안 켜짐)
            int phase_this_tick = (traj_cnt < dual_arm_phase_trajectory.size())
                                       ? dual_arm_phase_trajectory(traj_cnt) : task_phase;

            if (phase_this_tick == PHASE_GRASP_TO_PLACE || phase_this_tick == PHASE_RELEASE) {
                // ===== 어드미턴스 제어 (파지~내려놓기 구간): x,y 위치추종 + z 힘추종 =====
                // x_cmd(위치)를 매 tick 새로 계산해 IK로 q_cmd를 구하고, 그 q_cmd를 이번 tick의
                // 목표 관절각/속도/가속도로 그대로 쓴다 (기존 임피던스처럼 토크에 더하는 방식이 아님).
                pinocchio::forwardKinematics(model, data, dual_arm_jointp_vec);
                pinocchio::updateFramePlacements(model, data);
                Vector3d xL_actual = data.oMf[l_EE].translation();
                Vector3d xR_actual = data.oMf[r_EE].translation();
                Matrix3d RL_actual = data.oMf[l_EE].rotation();
                Matrix3d RR_actual = data.oMf[r_EE].rotation();

                // 사전 계획된 desired trajectory (x_d, ẋ_d, ẍ_d) 조회
                Vector3d xL_d(dual_arm_cart_target_trajectory(traj_cnt,0), dual_arm_cart_target_trajectory(traj_cnt,1), dual_arm_cart_target_trajectory(traj_cnt,2));
                Vector3d xR_d(dual_arm_cart_target_trajectory(traj_cnt,3), dual_arm_cart_target_trajectory(traj_cnt,4), dual_arm_cart_target_trajectory(traj_cnt,5));
                Vector3d xL_d_dot(dual_arm_cart_target_vel_trajectory(traj_cnt,0), dual_arm_cart_target_vel_trajectory(traj_cnt,1), dual_arm_cart_target_vel_trajectory(traj_cnt,2));
                Vector3d xR_d_dot(dual_arm_cart_target_vel_trajectory(traj_cnt,3), dual_arm_cart_target_vel_trajectory(traj_cnt,4), dual_arm_cart_target_vel_trajectory(traj_cnt,5));
                Vector3d xL_d_ddot(dual_arm_cart_target_acc_trajectory(traj_cnt,0), dual_arm_cart_target_acc_trajectory(traj_cnt,1), dual_arm_cart_target_acc_trajectory(traj_cnt,2));
                Vector3d xR_d_ddot(dual_arm_cart_target_acc_trajectory(traj_cnt,3), dual_arm_cart_target_acc_trajectory(traj_cnt,4), dual_arm_cart_target_acc_trajectory(traj_cnt,5));

                // PHASE_GRASP_TO_PLACE 진입 첫 tick: 불연속 방지를 위해 실제 현재 상태로 초기화.
                // y(스퀴즈)는 y_d(t)(이송 기준 경로) 대비 현재 오프셋을 deltaY 시작값으로 잡는다.
                if (!admittance_initialized) {
                    xL_cmd = xL_actual;
                    xR_cmd = xR_actual;
                    xL_cmd_dot.setZero();
                    xR_cmd_dot.setZero();
                    q_cmd_prev = dual_arm_jointp_vec;
                    q_cmd_dot_prev.setZero();
                    deltaYL = xL_actual(1) - xL_d(1);
                    deltaYR = xR_actual(1) - xR_d(1);
                    admittance_initialized = true;
                }

                // PHASE_RELEASE 진입 첫 tick: ticks_in_release를 0부터 다시 세기 시작.
                // (PHASE_GRASP_TO_PLACE 쪽은 위 admittance_initialized 블록과 동일한 패턴)
                if (phase_this_tick == PHASE_RELEASE) {
                    if (!release_initialized) {
                        ticks_in_release = 0;
                        release_initialized = true;
                    } else {
                        ticks_in_release++;
                    }
                } else {
                    release_initialized = false;  // GRASP_TO_PLACE 동안은 항상 리셋 상태로 유지
                }
                const double release_ramp = phase_this_tick == PHASE_RELEASE
                    ? std::min(1.0, (double)ticks_in_release / RELEASE_RAMP_TICKS)
                    : 0.0;
                const double target_fd_left  = ADMITTANCE_FD_Y_LEFT  * (1.0 - release_ramp);
                const double target_fd_right = ADMITTANCE_FD_Y_RIGHT * (1.0 - release_ramp);

                // F/T: 접촉 노이즈 완화 위해 기존과 동일하게 10Hz LPF 적용 후 world frame으로 변환
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
                Vector3d F_ext_L = RL_actual * left_ft_force_lpf;
                Vector3d F_ext_R = RR_actual * right_ft_force_lpf;

                // 동적 스퀴즈축(wrist-compliance-port 재작업, box-lifting 이식): 이 branch는 Waist를
                // (box-lifting과 달리) 자유롭게 두므로, Waist가 중립에서 벗어나면 두 grip 지점을 잇는
                // 실제 물리적 축이 world-Y에서 함께 회전한다 - 고정 world-Y 투영은 이 경우 스퀴즈력을
                // 조용히 잘못 계산하는 잠재적 정합성 문제였다(이번 손목 재작업 자체의 버그는 아니지만,
                // Task 1의 L_grip_frame/R_grip_frame이 이 축을 계산할 깨끗한 프레임을 처음 제공해 드러남).
                // L_grip_frame이 R_grip_frame보다 +y쪽에 마운트되므로(Task 1), squeeze_axis는 기존
                // world-Y와 같은 방향을 가리킨다 - Waist=0에서는 수치적으로 거의 동일하고, Waist가
                // 회전하면 그만큼 같이 회전해 실제 압착 방향을 따라간다.
                Vector3d squeeze_axis = (data.oMf[l_grip_EE].translation() - data.oMf[r_grip_EE].translation()).normalized();

                // 손목 면정렬 판정: 스퀴즈(동적 축) 방향으로 충분히 접촉하고 있고(compressive_force),
                // F/T 잔여 토크가 임계값 이하로 일정 시간 유지되면 "손목이 소프트 PD로 면에 안착했다"고 보고
                // wrist_alignment_ready를 래치한다. 이 블록은 토크를 읽어 판정만 하며 손목 관절을 직접
                // 명령하지 않는다(손목은 Task 2의 Kp=40 PD가 목표궤적(=0)을 추종하며 물리적으로 정렬).
                // 왼팔은 +스퀴즈축(+10N 목표), 오른팔은 -스퀴즈축(-10N 목표)으로 서로를 향해 조이므로
                // 압착 성분은 F_ext_L·squeeze_axis, -F_ext_R·squeeze_axis이다(기존 F_ext_L(1)/-F_ext_R(1)과
                // 동일한 부호 규약, 축만 world-Y 고정에서 동적으로 바뀜).
                const double compressive_force_L =  F_ext_L.dot(squeeze_axis);
                const double compressive_force_R = -F_ext_R.dot(squeeze_axis);
                if (!wrist_alignment_ready) {
                    const bool alignment_contact =
                        compressive_force_L >= WRIST_ALIGN_CONTACT_FORCE &&
                        compressive_force_R >= WRIST_ALIGN_CONTACT_FORCE;
                    const bool alignment_torque_ok =
                        std::abs(left_ft_torque_lpf(0))  <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                        std::abs(left_ft_torque_lpf(2))  <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                        std::abs(right_ft_torque_lpf(0)) <= WRIST_ALIGN_TORQUE_THRESHOLD &&
                        std::abs(right_ft_torque_lpf(2)) <= WRIST_ALIGN_TORQUE_THRESHOLD;
                    if (alignment_contact && alignment_torque_ok) {
                        wrist_alignment_ticks++;
                        if (wrist_alignment_ticks >= WRIST_ALIGN_HOLD_TICKS) {
                            wrist_alignment_ready = true;
                            ROS_INFO("Wrist face alignment ready: qL=%.3f qR=%.3f rad", dual_arm_jointp[7], dual_arm_jointp[13]);
                        }
                    } else {
                        wrist_alignment_ticks = 0;
                    }
                }

                // x,z: 위치추종 admittance (우항 0) / y(스퀴즈 방향): 힘추종 admittance (K_y=0, 목표힘 ADMITTANCE_FD_Y_*)
                // 이 로봇은 objL/objR이 obj ± (0,grasp_offset,0)로 world Y축 양쪽에서 마주보고 조이는
                // 구조라 스퀴즈 방향이 world Y다 (world Z는 들어올리기/이송 높이라 위치추종 대상).
                Vector3d xL_cmd_ddot = Vector3d::Zero(), xR_cmd_ddot = Vector3d::Zero();  // index 1(y)은 아래서 별도(deltaY)로 처리
                for (int k : {0, 2}) {   // x, z
                    xL_cmd_ddot(k) = xL_d_ddot(k) - (Da_left[k]*(xL_cmd_dot(k)-xL_d_dot(k))  + Ka_left[k]*(xL_cmd(k)-xL_d(k)))   / Ma_left[k];
                    xR_cmd_ddot(k) = xR_d_ddot(k) - (Da_right[k]*(xR_cmd_dot(k)-xR_d_dot(k)) + Ka_right[k]*(xR_cmd(k)-xR_d(k))) / Ma_right[k];
                }
                // y(스퀴즈): deltaY(y_d(t) 대비 순응 변위)만 K=0 힘추종 법칙을 따른다. 이송 자체는
                // y_d(t)가 담당하므로, 스퀴즈 축이 곧 이송 방향인 이 로봇 기하에서도 물체가 실제로
                // 옮겨진다 (deltaY는 그 위에 얹히는 작은 압착 보정일 뿐). 힘 오차항은 위 compressive_force와
                // 동일하게 world-Y 고정 대신 동적 squeeze_axis에 투영한 성분을 쓴다 - 이는 손목 정렬
                // 게이트뿐 아니라 admittance 법칙 자체의 입력이 바뀌는 실제 동작 변화다(순수 리팩터링 아님).
                double deltaYL_ddot = (F_ext_L.dot(squeeze_axis) - target_fd_left  - Da_left[1]*xL_cmd_dot(1))  / Ma_left[1];
                double deltaYR_ddot = (F_ext_R.dot(squeeze_axis) - target_fd_right - Da_right[1]*xR_cmd_dot(1)) / Ma_right[1];

                // Euler 적분으로 command(x_cmd) 생성
                xL_cmd_dot += xL_cmd_ddot * SAMPLING_TIME;
                xL_cmd     += xL_cmd_dot  * SAMPLING_TIME;
                xR_cmd_dot += xR_cmd_ddot * SAMPLING_TIME;
                xR_cmd     += xR_cmd_dot  * SAMPLING_TIME;

                // deltaY도 Euler 적분. K=0이라 되돌리는 힘이 없어 접촉 순간유실 등으로 힘 오차가
                // 계속 크게 남으면 무한정 커질 수 있어(첫 검증에서 실제 발산 확인) 속도/변위를 하드 리밋.
                xL_cmd_dot(1) += deltaYL_ddot * SAMPLING_TIME;
                xR_cmd_dot(1) += deltaYR_ddot * SAMPLING_TIME;
                xL_cmd_dot(1) = std::min(std::max(xL_cmd_dot(1), -Y_CMD_VEL_LIMIT), Y_CMD_VEL_LIMIT);
                xR_cmd_dot(1) = std::min(std::max(xR_cmd_dot(1), -Y_CMD_VEL_LIMIT), Y_CMD_VEL_LIMIT);
                deltaYL += xL_cmd_dot(1) * SAMPLING_TIME;
                deltaYR += xR_cmd_dot(1) * SAMPLING_TIME;
                deltaYL = std::min(std::max(deltaYL, -Y_CMD_MAX_DISP), Y_CMD_MAX_DISP);
                deltaYR = std::min(std::max(deltaYR, -Y_CMD_MAX_DISP), Y_CMD_MAX_DISP);
                xL_cmd(1) = xL_d(1) + deltaYL;
                xR_cmd(1) = xR_d(1) + deltaYR;

                // x_cmd -> IK (직전 q_cmd로 웜스타트, 매 tick 목표가 미세하게만 움직여 빠르게 수렴 예상)
                VectorXd q_cmd(DoF);
                dualarm.SolveIK_Position(model, data, l_EE, r_EE, xL_cmd, xR_cmd, q_cmd_prev, q_cmd);

                // 관절 속도/가속도: 온라인이라 미래 샘플이 없어 후진차분 사용 (기존 코드의 중심차분과 다름)
                VectorXd q_cmd_dot  = (q_cmd - q_cmd_prev) / SAMPLING_TIME;
                VectorXd q_cmd_ddot = (q_cmd_dot - q_cmd_dot_prev) / SAMPLING_TIME;

                for (int i = 0; i < DoF; i++) {
                    dual_arm_targetp[i] = q_cmd(i);
                    dual_arm_targetv[i] = q_cmd_dot(i);
                }
                dualarm.PDController(dual_arm_targetp, dual_arm_jointp, dual_arm_targetv, dual_arm_jointv, PD_acc);
                for (int i = 0; i < DoF; i++) {
                    dual_arm_targeta_vec(i) = q_cmd_ddot(i) + PD_acc[i];
                }

                q_cmd_prev     = q_cmd;
                q_cmd_dot_prev = q_cmd_dot;
            }
            else {
                admittance_initialized = false;  // GRASP_TO_PLACE 밖 -> 다음 진입에 대비해 리셋

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
            }

            task_phase = phase_this_tick;
            // 손목 정렬 게이트: PHASE_GRASP_TO_PLACE에 진입해도 손목이 소프트 PD로 면에 안착(정렬)해
            // wrist_alignment_ready가 될 때까지는 traj_cnt를 진행시키지 않아, 파지점(objL/objR)에서
            // 스퀴즈를 유지하며 대기하고 정렬이 끝난 뒤에야 들어올리기/이송 구간으로 넘어간다.
            // (이 branch의 phase 전환은 사전 태깅된 궤적 행 진행으로 이뤄지므로, "전환 조건"을 거는 것은
            //  곧 이 행 진행(traj_cnt++)을 게이팅하는 것과 같다.)
            if (!(phase_this_tick == PHASE_GRASP_TO_PLACE && !wrist_alignment_ready)) {
                traj_cnt++;
            }
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
                admittance_initialized = false;  // 다음 PHASE_GRASP_TO_PLACE 진입에 대비해 리셋

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

        // 어드미턴스 제어는 위 PHASE_GRASP_TO_PLACE 분기 안에서 이미 dual_arm_targetp/targetv/targeta를
        // q_cmd/q_cmd_dot/q_cmd_ddot로 직접 채웠으므로, 여기서는 그 값 그대로 RNEA에 넘기기만 하면 된다
        // (예전 임피던스처럼 토크 레벨에서 별도로 더해줄 필요 없음).

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
            wrist_yaw_l_joint_msg.data      = dual_arm_targetp[7];
            wrist_pitch_l_joint_msg.data    = dual_arm_targetp[8];
            shoulder_pitch_r_joint_msg.data = dual_arm_targetp[9];
            shoulder_roll_r_joint_msg.data  = dual_arm_targetp[10];
            shoulder_yaw_r_joint_msg.data   = dual_arm_targetp[11];
            elbow_r_joint_msg.data          = dual_arm_targetp[12];
            wrist_yaw_r_joint_msg.data      = dual_arm_targetp[13];
            wrist_pitch_r_joint_msg.data    = dual_arm_targetp[14];

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
            wrist_pitch_l_joint_msg.data    = target_torque[8];
            shoulder_pitch_r_joint_msg.data = target_torque[9];
            shoulder_roll_r_joint_msg.data  = target_torque[10];
            shoulder_yaw_r_joint_msg.data   = target_torque[11];
            elbow_r_joint_msg.data          = target_torque[12];
            wrist_yaw_r_joint_msg.data      = target_torque[13];
            wrist_pitch_r_joint_msg.data    = target_torque[14];

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
        dual_armjoint12_pub.publish(wrist_yaw_l_joint_msg);
        dual_armjoint13_pub.publish(wrist_yaw_r_joint_msg);
        dual_armjoint14_pub.publish(wrist_pitch_l_joint_msg);
        dual_armjoint15_pub.publish(wrist_pitch_r_joint_msg);

        
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