#include "arm_function.cpp"
#include <thread>
#include <chrono>

using namespace std;
using namespace Eigen;

//---------------------------- Program start ---------------------------------//
/*############################################################################*/
/*############################################################################*/
//---------------------------- CallBack ---------------------------------//
void msgCallbackArmCmdMode_sim(const std_msgs::Float64::ConstPtr& msg)
{
    arm_cmd_mode = msg->data;
    traj_init = true;
}

void msgCallbackArmJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (int i = 0; i < DoF; i++) {
		arm_jointp[i] = msg->position[i];
        arm_jointv[i] = msg->velocity[i];
	}
    first_callback = true;
}

void msgCallbackMobileState(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (int i = 0; i < 3; i++)	{
        Mobile_p[i] = msg->position[i+6];
        Mobile_v[i] = msg->velocity[i+6];
    }
    first_callback = true;
}

void msgCallbackArmCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for (int i = 0; i < DoF; i++) {
            arm_commandp[i] = msg-> data[i];
        }
        traj_init = true;
    }

void msgCallbackMobileCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        for (int i = 0; i < 2; i++) {
            mobile_commandp[i] = msg-> data[i];
        }
        t = true;
    }

void msgCallbackArucoPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose[0] = msg-> pose.position.x;
        pose[1] = msg-> pose.position.y;
        pose[2] = msg-> pose.position.z;
        pose[3] = msg-> pose.orientation.x;
        pose[4] = msg-> pose.orientation.y;
        pose[5] = msg-> pose.orientation.z;
        pose[6] = msg-> pose.orientation.w;
        p = true;
    }

void msgCallbackArucoDetectionCmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        marker = msg-> data[0];
        a = true;
    }

//---------------------------- Main ---------------------------------//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aidin_control_main");
    ros::NodeHandle nh;

    #if ARMCTRLMODE == POSITION
        //joint position command publisher
        ros::Publisher armjoint1_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint1_position_controller/command", 100);
        ros::Publisher armjoint2_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint2_position_controller/command", 100);
        ros::Publisher armjoint3_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint3_position_controller/command", 100);
        ros::Publisher armjoint4_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint4_position_controller/command", 100);
        ros::Publisher armjoint5_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint5_position_controller/command", 100);
        ros::Publisher armjoint6_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint6_position_controller/command", 100);
        ros::Publisher x_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint7_position_controller/command", 100);
        ros::Publisher y_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint8_position_controller/command", 100);
        ros::Publisher z_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint9_position_controller/command", 100);

    #else
        //joint torque command publisher
        ros::Publisher armjoint1_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint1_effort_controller/command", 100);
        ros::Publisher armjoint2_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint2_effort_controller/command", 100);
        ros::Publisher armjoint3_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint3_effort_controller/command", 100);
        ros::Publisher armjoint4_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint4_effort_controller/command", 100);
        ros::Publisher armjoint5_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint5_effort_controller/command", 100);
        ros::Publisher armjoint6_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint6_effort_controller/command", 100);
        ros::Publisher x_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint7_position_controller/command", 100);
        ros::Publisher y_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint8_position_controller/command", 100);
        ros::Publisher z_joint_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/joint9_position_controller/command", 100);

        ros::Publisher X_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_X", 100);
        ros::Publisher Y_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_Y", 100);
        ros::Publisher Z_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_Z", 100);
        ros::Publisher q_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_q", 100);
        ros::Publisher qx_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_qx", 100);
        ros::Publisher qy_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_qy", 100);
        ros::Publisher qz_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/arm_traj_pose_quat_qz", 100);

    #endif
    //joint position subscriber
	ros::Subscriber sub_joint_angle = nh.subscribe("/aidin_arm/joint_states", 100, msgCallbackArmJointState);
    ros::Subscriber sub_joint_cmd = nh.subscribe("/aidin_arm/ArmCmd_sim", 100, msgCallbackArmCmd);
    ros::Subscriber sub_arm_cmd_mode = nh.subscribe("/aidin_arm/ArmCmdMode_sim", 100, msgCallbackArmCmdMode_sim);

    ros::Subscriber sub_aruco_pose = nh.subscribe("aruco_single/pose", 100, msgCallbackArucoPose);

    ros::Subscriber sub_mobile_position = nh.subscribe("/aidin_arm/joint_states", 100, msgCallbackMobileState);
    ros::Subscriber sub_mobile_cmd = nh.subscribe("/aidin_arm/MobileCmd_sim", 100, msgCallbackMobileCmd);

    ros::Subscriber sub_aruco_detection_cmd = nh.subscribe("/aidin_arm/ArucoDetectionCmd_sim", 100, msgCallbackArucoDetectionCmd);

    ros::Rate loop_rate(1000);
   	ros::spinOnce();

    //messages
    std_msgs::Float64 armjoint1_msg, armjoint2_msg, armjoint3_msg, armjoint4_msg, armjoint5_msg, armjoint6_msg, z_joint_msg, x_joint_msg, y_joint_msg, X_msg, Y_msg, Z_msg, q_msg, qx_msg, qy_msg, qz_msg;

    while(ros::ok())
    {
        ///////////// Aruco marker Pose (Euler) //////////////////
        Quaterniond q_aruco(pose[6], pose[3], pose[4], pose[5]); 
        arm.Qua2Euler(q_aruco, eul_aruco);
        p_aruco(0) = pose[0]; //X
        p_aruco(1) = pose[1]; //Y
        p_aruco(2) = pose[2]; //Z
        p_aruco(3) = eul_aruco(0); //Roll 
        p_aruco(4) = eul_aruco(1); //Pitch
        p_aruco(5) = eul_aruco(2); //Yaw

        arm.ab_T_link5(arm_jointp, T05_act); //arm_base -> link5 TF
        arm.link5_T_cam(T5cam_act); //link5 -> cam TF

        T0cam_act = T05_act * T5cam_act; //arm_base -> cam TF

        arm.Tf2PoseEul(T0cam_act, T0cam);

        arm.PoseEul2Tf(p_aruco, Tcamaruco_act); //cam -> aruco TF

        T0aruco_act = T0cam_act * Tcamaruco_act; //arm_base -> aruco TF

        arm.mb_T_ab(Tmb0_act); //mobile_base -> arm_base TF

        Tmbaruco_act = Tmb0_act * T0aruco_act; //mobile_base -> aruco TF

        arm.Tf2PoseEul(T0aruco_act, ab_aruco); // aruco TF -> aruco pose (reference frame = arm_base)

        arm.Tf2PoseEul(Tmbaruco_act, mb_aruco); // aruco TF -> aruco pose (reference frame = mobile_base)

        ///////////// Mobile_Robot ////////////////////////
        if(t == true)
        {
            for (int i = 0; i < 2; i++) {
                mobile_targetp[i] = mobile_commandp[i];
            }
            for (int i = 0; i < 3; i++) {
                mobile_gazebop[i] = Mobile_p[i];
            }

            arm.MobileTrajectory_Z(mobile_gazebop, mobile_targetp, mobile_out_Z);
            arm.MobileTrajectory(mobile_gazebop, mobile_targetp, mobile_out);
            cnt = 0;
            cnt_z = 0;
            mobile_commandp[2] = mobile_out_Z(mobile_out_Z.rows()-1, 2);
            t = false;
        }

        else if(cnt_z < mobile_out_Z.rows())
        {
            mobile_act[2] = mobile_out_Z(cnt_z, 2); 
            cnt_z++;
        }


        else if(cnt_z >= mobile_out_Z.rows() && cnt < mobile_out.rows())
        {
            for (int i = 0; i < 2; i++)
            {
                mobile_act[i] = mobile_out(cnt, i); 
            }
            cnt++;
        }

        else
        {
            for (int i = 0; i < 3; i++) 
            {
                mobile_act[i] = mobile_commandp[i];
            }
        }

        ///////////// Aidin_arm ////////////////////////
        arm.forwardKinematics(arm_jointp, T06_act); //arm_base -> EE TF
        arm.Tf2PoseEul(T06_act, pose_ee); // arm_base -> EE

        arm.forwardKinematics(arm_commandp, T06_cmd);

        arm.Joint2Cartesian(arm_jointp, arm_pose_act);

        arm_pose_filtered = arm_pose_act;
        arm_pose_error = arm_pose_act - arm_pose_before;
        for (int i = 3; i < 6; i++)
        {
            if (arm_pose_error(i) >= PI) {
                arm_pose_filtered(i) -= 2*PI;
            }
            else if (arm_pose_error(i) < -PI) {
                arm_pose_filtered(i) += 2*PI;
            }
        }

        arm.PoseEuler2Quat(arm_pose_filtered, arm_pose_quat_act);

        // Jointvelocity LowpassFilter
        for (int i = 0; i < DoF; i++)
        {
            arm_jointv_lpf[i] = arm.LowPassFilter(arm_jointv[i], arm_jointv_before[i], 30);
            arm_jointv_before[i] = arm_jointv_lpf[i];
        }

        // for (int i = 0; i < DoF; i++)
        // {
        //     arm_jointp_lpf[i] = arm.LowPassFilter(arm_jointp[i], arm_jointp_before[i], 30);
        //     arm_jointp_before[i] = arm_jointp_lpf[i];
        // }

        // for (int i = 0; i < 6; i++)
        // {
        //     mb_aruco_lpf(i) = arm.LowPassFilter(mb_aruco(i), mb_aruco_before(i), 1);
        //     mb_aruco_before(i) = mb_aruco_lpf(i);
        // }


        ///////////////////////////////////////////////// Start Trajectory Calculation ////////////////////////////////////////////////////////////////

        if (arm_cmd_mode == 0 && d == true) // Default mode
        {
            for (int i = 0; i < DoF; i++)
            {
                arm_targetp[i] = arm_initp[i];
                // arm_targetp[i] = arm_commandp[i];
            }
        }

        else if (arm_cmd_mode == 1) // cmd angle mode
        {
            if (traj_init == true) // set when cmd angle come in, trajectory generated
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_jointp_initial[i] = arm_jointp[i];
                }
                arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory, arm_jointv_trajectory);
                // JointTrajectory(arm_jointp_initial, arm_commandp, arm_jointp_trajectory);
                traj_cnt = 0;
                traj_init = false;
            }
            else if (traj_cnt < arm_jointp_trajectory.rows()) // this portion must not run when no cmd
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_targetp[i] = arm_jointp_trajectory(traj_cnt, i);
                }
                traj_cnt++;
            }
            else
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_targetp[i] = arm_commandp[i];
                }
            }
        }

        else if(arm_cmd_mode == 2) // cmd Cartesian space trajectory mode
        {
            if(traj_init == true) // set when cmd angle come in, trajectory generated
            {
                #if TRAJMODE == EULER
                    pose_ini = arm_pose_filtered;
                    arm.Joint2Cartesian(arm_commandp, pose_cmd);
                    arm.CartesianTrajectoryEuler(pose_ini, pose_cmd, pose_out);
                    traj_size = pose_out.rows();
                #elif TRAJMODE == QUAT
                    pose_quat_ini = arm_pose_quat_act;
                    arm.Joint2CartesianQuat(arm_commandp, pose_quat_cmd);
                    arm.CartesianTrajectoryTrapezoidal(pose_quat_ini, pose_quat_cmd, pose_quat_out);
                    // arm.CartesianTrajectoryQuat(pose_quat_ini, pose_quat_cmd, pose_quat_out);
                    traj_size = pose_quat_out.rows();
                #endif

                traj_cnt = 0;
                traj_init = false;
            }
            else if(traj_cnt < traj_size) // traj_cnt = 1 when initialize
            {
                #if TRAJMODE == EULER
                    for (int i = 0; i < DoF; i++)
                    {
                        arm_traj_pose(i) = pose_out(traj_cnt, i);
                    }
                    // arm.Inverse_Newton(arm_traj_pose, arm_jointp, arm_pose_filtered, arm_targetp); // IK-eul
                #elif TRAJMODE == QUAT
                    for (int i = 0; i < 7; i++)
                    {
                        arm_traj_pose_quat(i) = pose_quat_out(traj_cnt, i);
                    }
                    arm.PoseQuat2Euler(arm_traj_pose_quat, arm_traj_pose);
                    // arm.Inverse_Newton_Quat(arm_traj_pose_quat, arm_jointp, arm_targetp); // IK-quat
                #endif
                arm.inverseKinematics(arm_traj_pose, arm_jointp, arm_targetp); // IK-analytic
                traj_cnt++;
            }
            // else
            // {
            //     for (int i = 0; i < DoF; i++) {
            //         arm_targetp[i] = arm_commandp[i];
            //     }
            // }
            arm_traj_pose_quat_vel = (arm_traj_pose_quat - arm_traj_pose_quat_before)/0.001;
            arm_traj_pose_quat_before = arm_traj_pose_quat;
        }

        //////////////////////////////////////////////// Tracking ////////////////////////////////////////////////////////////////////////////
        if (p == true && marker == 0)
        {
            while (true) 
            {
                if (ma == true)
                {
                    
                    if (ab_aruco(0) >= 0.5) {
                        ab_aruco(0) = 0.5;
                    } else if (ab_aruco(0) <= 0.2) {
                        ab_aruco(0) = 0.2;
                    }

                    if (ab_aruco(1) >= 0.5) {
                        ab_aruco(1) = 0.5;
                    } else if (ab_aruco(1) <= 0.2) {
                        ab_aruco(1) = 0.2;
                    }

                    if (ab_aruco(2) >= 0.5) {
                        ab_aruco(2) = 0.5;
                    } else if (ab_aruco(2) <= 0.2) {
                        ab_aruco(2) = 0.2;
                    }

                  
                    mani_cmd_1(0) = ab_aruco(0);
                    mani_cmd_1(1) = ab_aruco(1);
                    mani_cmd_1(2) = ab_aruco(2); 
                    mani_cmd_1(3) = (static_cast<int>(ab_aruco(3)) + 90) * DEG2RAD;
                    mani_cmd_1(4) = static_cast<int>(ab_aruco(4)) * DEG2RAD;
                    mani_cmd_1(5) = static_cast<int>(ab_aruco(5)) * DEG2RAD;

                    arm.inverseKinematics(mani_cmd_1, arm_jointp, arm_commandp);

                    ma = false;
                    traj = true;
                    d = false;
                }

                else if (traj == true) 
                {
                    for (int i = 0; i < DoF; i++)
                    {
                        arm_jointp_initial[i] = arm_jointp[i];
                    }
                    arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory, arm_jointv_trajectory);
                    traj_cnt = 0;
                    traj = false;
                    tq = true;
                }

                else if (traj_cnt < arm_jointp_trajectory.rows()) 
                {
                    for (int i = 0; i < DoF; i++)
                    {
                        arm_targetp[i] = arm_jointp_trajectory(traj_cnt, i);
                    }
                    traj_cnt++;
                }

                else if (traj_cnt >= arm_jointp_trajectory.rows() && tq == true)
                {
                    ma = true;
                    tq = false;
                    traj_cnt = 0;
                    // arm_jointp_trajectory = MatrixXd::Zero(1,6);
                }

            }
        }


        //////////////////////////////////////////////// Motion Planning Cartesian Trajectory //////////////////////////////////////////////////////////////////////////////
        if(p==true && marker == 1)
        {   
            if(mo==true){
                mobile_commandp[0] = mb_aruco(0)-1;
                mobile_commandp[1] = 0;
                arm.MobileTrajectory(Mobile_p, mobile_commandp, mobile_out);
                cnt = 0;
                mo = false;
            }

            else if(cnt < mobile_out.rows()){
                for (int i = 0; i < 2; i++)
                {
                    mobile_act[i] = mobile_out(cnt, i); 
                }
            cnt++;
            }

            else if(mb_aruco(0) <= 1 && ma==true){
                mani_cmd_1(0) = ab_aruco(0)-0.5;
                mani_cmd_1(1) = static_cast<int>(ab_aruco(1));
                mani_cmd_1(2) = ab_aruco(2);
                mani_cmd_1(3) = static_cast<int>(ab_aruco(3)) * DEG2RAD;;
                mani_cmd_1(4) = static_cast<int>(ab_aruco(4)) * DEG2RAD;;
                mani_cmd_1(5) = static_cast<int>(ab_aruco(5)) * DEG2RAD;;
                arm.CartesianTrajectoryEuler(arm_pose_filtered, mani_cmd_1, pose_out_1);
                traj_cnt_1 = 0;
                traj_1 = true;
                d = false;
                ma = false;
            }

            else if(traj_cnt_1 < pose_out_1.rows()){
                for (int i = 0; i < DoF; i++)
                    {
                        arm_traj_pose_1(i) = pose_out_1(traj_cnt_1, i);
                    }
                arm.inverseKinematics(arm_traj_pose_1, arm_jointp, arm_targetp); 
                traj_cnt_1++;
            }

            else if(traj_cnt_1 >= pose_out_1.rows() && traj_1 == true ){
                arm.inverseKinematics(mani_cmd_1, arm_jointp, arm_jointp_initial);
                arm_commandp[0] = arm_jointp_initial[0];
                arm_commandp[1] = arm_jointp_initial[1];
                arm_commandp[2] = arm_jointp_initial[2];
                arm_commandp[3] = arm_jointp_initial[3];
                arm_commandp[4] = arm_jointp_initial[4];
                arm_commandp[5] = PI/2;
                arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory, arm_jointv_trajectory);
                traj_cnt_2 = 0;
                traj_2 = true;
                traj_1 = false;
            }

            else if(traj_cnt_2 < arm_jointp_trajectory.rows()){
                for (int i = 0; i < DoF; i++)
                    {
                         arm_targetp[i] = arm_jointp_trajectory(traj_cnt_2, i);
                    }
                traj_cnt_2++;
            }

            else if(traj_2 == true && traj_cnt_2 >= arm_jointp_trajectory.rows()){
                mani_cmd_2(0) = mani_cmd_1(0) + 0.4;
                mani_cmd_2(1) = mani_cmd_1(1);
                mani_cmd_2(2) = mani_cmd_1(2);
                mani_cmd_2(3) = mani_cmd_1(3) + PI/2;
                mani_cmd_2(4) = mani_cmd_1(4);
                mani_cmd_2(5) = mani_cmd_1(5);
                arm.CartesianTrajectoryEuler(arm_pose_filtered, mani_cmd_2, pose_out_2);
                traj_cnt_3 = 0;
                traj_3 = true;
                traj_2 = false;
            }

            else if(traj_cnt_3 < pose_out_2.rows()){
                for (int i = 0; i < DoF; i++)
                    {
                        arm_traj_pose_2(i) = pose_out_2(traj_cnt_3, i);
                    }
                arm.inverseKinematics(arm_traj_pose_2, arm_jointp, arm_targetp); 
                traj_cnt_3++;
            }

            else if(traj_3 == true && traj_cnt_3 >= pose_out_2.rows()){
                mani_cmd_3(0) = mani_cmd_1(0);
                mani_cmd_3(1) = mani_cmd_1(1);
                mani_cmd_3(2) = mani_cmd_1(2);
                mani_cmd_3(3) = mani_cmd_1(3) + PI/2;
                mani_cmd_3(4) = mani_cmd_1(4);
                mani_cmd_3(5) = mani_cmd_1(5);
                arm.CartesianTrajectoryEuler(arm_pose_filtered, mani_cmd_3, pose_out_3);
                traj_cnt_4 = 0;
                traj_4 = true;
                traj_3 = false;
            }

            else if(traj_cnt_4 < pose_out_3.rows()){
                for (int i = 0; i < DoF; i++)
                    {
                        arm_traj_pose_3(i) = pose_out_3(traj_cnt_4, i);
                    }
                arm.inverseKinematics(arm_traj_pose_3, arm_jointp, arm_targetp); 
                traj_cnt_4++;
            }

            else if(traj_cnt_4 >= pose_out_3.rows() && traj_4 == true ){
                arm.inverseKinematics(mani_cmd_3, arm_jointp, arm_jointp_initial_1);
                arm.JointTrajectoryTrapezoidal(arm_jointp_initial_1, arm_initp_90, arm_jointp_trajectory_1, arm_jointv_trajectory);
                traj_cnt_5 = 0;
                traj_5 = true;
                traj_4 = false;
            }

            else if(traj_cnt_5 < arm_jointp_trajectory_1.rows()){
                for (int i = 0; i < DoF; i++)
                    {
                         arm_targetp[i] = arm_jointp_trajectory_1(traj_cnt_5, i);
                    }
                traj_cnt_5++;
            }

            else if(traj_cnt_5 >= arm_jointp_trajectory_1.rows() && traj_5 == true )
            {
                for (int i = 0; i < 2; i++) {
                    mobile_commandp[i] = 0; 
                }
                for (int i = 0; i < 3; i++) {
                    mobile_gazebop[i] = Mobile_p[i];
                }

                arm.MobileTrajectory_Z(mobile_gazebop, mobile_commandp, mobile_out_Z);
                arm.MobileTrajectory(mobile_gazebop, mobile_commandp, mobile_out_1);
                cnt_1 = 0;
                cnt_z = 0;
                mobile_commandp[2] = mobile_out_Z(mobile_out_Z.rows()-1, 2);
                traj_5 = false;
            }

            else if(cnt_z < mobile_out_Z.rows())
            {
                mobile_act[2] = mobile_out_Z(cnt_z, 2); 
                cnt_z++;
            }


            else if(cnt_z >= mobile_out_Z.rows() && cnt_1 < mobile_out_1.rows())
            {
                for (int i = 0; i < 2; i++)
                {
                    mobile_act[i] = mobile_out_1(cnt_1, i); 
                }
                cnt_1++;
            }

        }

        //////////////////////////////////////////////// Motion Planning Joint Trajectory //////////////////////////////////////////////////////////////////////////////

        if(p==true && marker == 2)
        {   
            if(mo==true){
                mobile_commandp[0] = mb_aruco(0)-1;
                mobile_commandp[1] = 0;
                arm.MobileTrajectory(Mobile_p, mobile_commandp, mobile_out);
                cnt = 0;
                mo = false;
            }

            else if(cnt < mobile_out.rows()){
                for (int i = 0; i < 2; i++)
                {
                    mobile_act[i] = mobile_out(cnt, i); 
                }
            cnt++;
            }

            else if(mb_aruco(0) <= 1 && ma==true){
                mani_cmd_1(0) = ab_aruco(0) - 0.1;
                mani_cmd_1(1) = ab_aruco(1);
                mani_cmd_1(2) = ab_aruco(2); 
                mani_cmd_1(3) = (static_cast<int>(ab_aruco(3)) + 90) * DEG2RAD;
                mani_cmd_1(4) = static_cast<int>(ab_aruco(4)) * DEG2RAD;
                mani_cmd_1(5) = static_cast<int>(ab_aruco(5)) * DEG2RAD;
                // mani_cmd_1(3) = ab_aruco(3) * DEG2RAD;
                // mani_cmd_1(4) = ab_aruco(4) * DEG2RAD;
                // mani_cmd_1(5) = ab_aruco(5) * DEG2RAD;

                double ee_pos = sqrt(pow(ab_aruco(0), 2) + pow(ab_aruco(1), 2) + pow(ab_aruco(2), 2));
			    double ee_pos_limit = 0.108 + 0.35 + sqrt(pow(0.1, 2) + pow(0.4, 2)) + 0.1;
			    if(ee_pos > ee_pos_limit)
                  {
                    cout << "Out of Configuration!" << '\n';
                    ma=false;
                    continue;
                  }
			    else
                  {
                    arm.inverseKinematics(mani_cmd_1, arm_jointp, arm_commandp);
                    ma = false;
                    traj = true;
                    d = false;
                  }
            }

            else if(traj == true) // set when cmd angle come in, trajectory generated
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_jointp_initial[i] = arm_jointp[i];
                }
                arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory, arm_jointv_trajectory);
                // JointTrajectory(arm_jointp_initial, arm_commandp, arm_jointp_trajectory);
                traj_cnt = 0;
                traj_1 = true;
                traj = false;
            }

            else if(traj_cnt < arm_jointp_trajectory.rows()) // this portion must not run when no cmd
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_targetp[i] = arm_jointp_trajectory(traj_cnt, i);
                }
                traj_cnt++;
            }

            else if(traj_cnt >= arm_jointp_trajectory.rows() && traj_1 == true ) // set when cmd angle come in, trajectory generated
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_jointp_initial[i] = arm_jointp[i];
                }
                for (int i = 0; i < DoF; i++)
                {
                    arm_commandp[i] = arm_initp_90[i];
                }
                arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory_1, arm_jointv_trajectory);
                traj_cnt_1 = 0;
                traj_2 = true;
                traj_1 = false;
            }

            else if(traj_cnt_1 < arm_jointp_trajectory_1.rows()) // this portion must not run when no cmd
            {
                for (int i = 0; i < DoF; i++)
                {
                    arm_targetp[i] = arm_jointp_trajectory_1(traj_cnt_1, i);
                }
                traj_cnt_1++;
            }

            else if(traj_cnt_1 >= arm_jointp_trajectory_1.rows() && traj_2 == true )
            {
                for (int i = 0; i < 2; i++) {
                    mobile_commandp[i] = 0; 
                }
                for (int i = 0; i < 3; i++) {
                    mobile_gazebop[i] = Mobile_p[i];
                }

                arm.MobileTrajectory_Z(mobile_gazebop, mobile_commandp, mobile_out_Z);
                arm.MobileTrajectory(mobile_gazebop, mobile_commandp, mobile_out_1);
                cnt_1 = 0;
                cnt_z = 0;
                mobile_commandp[2] = mobile_out_Z(mobile_out_Z.rows()-1, 2);
                traj_2 = false;
            }

            else if(cnt_z < mobile_out_Z.rows())
            {
                mobile_act[2] = mobile_out_Z(cnt_z, 2); 
                cnt_z++;
            }


            else if(cnt_z >= mobile_out_Z.rows() && cnt_1 < mobile_out_1.rows())
            {
                for (int i = 0; i < 2; i++)
                {
                    mobile_act[i] = mobile_out_1(cnt_1, i); 
                }
                cnt_1++;
            }

        }

       ///////////////////////////////////////////////  First Move /////////////////////////////////////////////////////////////////////////////


        if(marker == 3)
        {
            d = false;
            for(int i = 0; i < DoF; i++) {
                arm_jointp_initial[i] = arm_jointp[i];
            }
            arm_commandp[0] = PI/2;
            arm_commandp[1] = -PI;
            arm_commandp[2] = PI;
            arm_commandp[3] = 0;
            arm_commandp[4] = 0;
            arm_commandp[5] = 0;

            arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory_1, arm_jointv_trajectory);
            cnt_a = 0;
            marker = 0;
            b =true;
        }
        else if(cnt_a < arm_jointp_trajectory_1.rows() && p == false)
        {
            for (int i = 0; i < DoF; i++) {
                    arm_targetp[i] = arm_jointp_trajectory_1(cnt_a, i);
            }
            cnt_a++;
        }

        else if(cnt_a >= arm_jointp_trajectory_1.rows() && b == true)
        {
            for(int i = 0; i < DoF; i++) {
                arm_jointp_initial[i] = arm_jointp[i];
            }
            arm_commandp[0] = -PI/2;
            arm_commandp[1] = -PI;
            arm_commandp[2] = PI;
            arm_commandp[3] = 0;
            arm_commandp[4] = 0;
            arm_commandp[5] = 0;

            arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory_2, arm_jointv_trajectory);
            cnt_b = 0;
            b = false;
            c = true;
        }
        else if(cnt_b < arm_jointp_trajectory_2.rows() && p == false)
        {
            
            for (int i = 0; i < DoF; i++) {
                arm_targetp[i] = arm_jointp_trajectory_2(cnt_b, i);
            }
            cnt_b++;
        }

        else if(cnt_b >= arm_jointp_trajectory_2.rows() &&  c == true)
        {
            for(int i = 0; i < DoF; i++) {
                arm_jointp_initial[i] = arm_jointp[i];
            }
            arm_commandp[0] = 0;
            arm_commandp[1] = -PI;
            arm_commandp[2] = PI;
            arm_commandp[3] = 0;
            arm_commandp[4] = 0;
            arm_commandp[5] = 0;

            arm.JointTrajectoryTrapezoidal(arm_jointp_initial, arm_commandp, arm_jointp_trajectory_3, arm_jointv_trajectory);
            cnt_c = 0;
            c = false;
        }
        else if(cnt_c < arm_jointp_trajectory_3.rows() && p == false)
        {
            
            for (int i = 0; i < DoF; i++) {
                arm_targetp[i] = arm_jointp_trajectory_3(cnt_c, i);
            }
            cnt_c++;
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        

        #if ARMCTRLMODE == POSITION
            armjoint1_msg.data = arm_targetp[0];
            armjoint2_msg.data = arm_targetp[1];
            armjoint3_msg.data = arm_targetp[2];
            armjoint4_msg.data = arm_targetp[3];
            armjoint5_msg.data = arm_targetp[4];
            armjoint6_msg.data = arm_targetp[5];
            x_joint_msg.data = mobile_act[0];
            y_joint_msg.data = mobile_act[1];
            z_joint_msg.data = mobile_act[2];

        #else
            arm.PIDController(arm_targetp, arm_jointp, arm_jointv_lpf, PID_torque);
            arm.GravityTorque(arm_jointp, gravity_torque);
            // arm.RNEA(arm_jointp, arm_jointv, PID_torque, vec3d_zeros, vec3d_zeros, vec3d_zeros, dynamic_torque);

            for (int i = 0; i < DoF; i++)
            {
                arm_target_torque[i] = PID_torque[i] + gravity_torque[i];
                // arm_target_torque[i] = dynamic_torque[i];
            }

            armjoint1_msg.data = arm_target_torque[0];
            armjoint2_msg.data = arm_target_torque[1];
            armjoint3_msg.data = arm_target_torque[2];
            armjoint4_msg.data = arm_target_torque[3];
            armjoint5_msg.data = arm_target_torque[4];
            armjoint6_msg.data = arm_target_torque[5];
            x_joint_msg.data = mobile_act[0];
            y_joint_msg.data = mobile_act[1];
            z_joint_msg.data = mobile_act[2];

            X_msg.data = arm_traj_pose_quat_vel(0);
            Y_msg.data = arm_traj_pose_quat_vel(1);
            Z_msg.data = arm_traj_pose_quat_vel(2);
            q_msg.data = arm_traj_pose_quat_vel(3);
            qx_msg.data = arm_traj_pose_quat_vel(4);
            qy_msg.data = arm_traj_pose_quat_vel(5);
            qz_msg.data = arm_traj_pose_quat_vel(6);

        #endif

        armjoint1_pub.publish(armjoint1_msg);
        armjoint2_pub.publish(armjoint2_msg);
        armjoint3_pub.publish(armjoint3_msg);
        armjoint4_pub.publish(armjoint4_msg);
        armjoint5_pub.publish(armjoint5_msg);
        armjoint6_pub.publish(armjoint6_msg);
        z_joint_pub.publish(z_joint_msg);
        x_joint_pub.publish(x_joint_msg);
        y_joint_pub.publish(y_joint_msg);

        X_pub.publish(X_msg);
        Y_pub.publish(Y_msg);
        Z_pub.publish(Z_msg);
        q_pub.publish(q_msg);
        qx_pub.publish(qx_msg);
        qy_pub.publish(qy_msg);
        qz_pub.publish(qz_msg);
        
        /////////////////////////////////
        ////////// print state //////////
        /////////////////////////////////

        cout << fixed;
        cout.precision(3);
        // cout << mobile_targetp[0] << setw(8) <<  mobile_targetp[1] << '\n';
        // cout << Mobile_p[0] << setw(8) <<  Mobile_p[1] << setw(8) <<  Mobile_p[2] << '\n';
        // cout << "CurRPY: " << arm_pose_act(3)*180/PI << setw(8) << arm_pose_act(4)*180/PI << setw(8) << arm_pose_act(5)*180/PI << '\n';
        // cout << "CurPos: " << arm_jointp[0]*180/PI << setw(8) << arm_jointp[1]*180/PI << setw(8) << arm_jointp[2]*180/PI << '\n';

        // cout << "EE Position: " << '\n' << "X,Y,Z : " << T06_act(0,3) << ' ' << T06_act(1,3) << ' ' << T06_act(2,3) << '\n';
        // cout << "Link5 Position: " << '\n' << "X,Y,Z : " << T05_act(0,3) << ' ' << T05_act(1,3) << ' ' << T05_act(2,3) << '\n';
        // cout << "Cam Position: " << '\n' << "X,Y,Z : " << T0cam_act(0,3) << ' ' << T0cam_act(1,3) << ' ' << T0cam_act(2,3) << '\n';
        // cout << "Aruco Position: " << '\n' << "X,Y,Z : " << Tcamaruco_act(0,3) << ' ' << Tcamaruco_act(1,3) << ' ' << Tcamaruco_act(2,3) << '\n';
        // cout << "Arm_base -> Aruco Position: " << '\n' << "X,Y,Z : " << T0aruco_act(0,3) << ' ' << T0aruco_act(1,3) << ' ' << T0aruco_act(2,3) << '\n';

        // cout << "X,Y,Z: " << pose[0] << ' ' << pose[1]  << ' ' << pose[2] <<'\n';
        // cout << pose[0] << ' ' << pose[1]  << ' ' << pose[2] << ' ' << pose[3] << ' ' << pose[4] << ' ' << pose[5] << ' ' << pose[6] <<'\n';
        // cout << p_aruco(0) << ' ' << p_aruco(1)  << ' ' << p_aruco(2) << ' ' << p_aruco(3) << ' ' << p_aruco(4) << ' ' << p_aruco(5) <<'\n';

        cout << ab_aruco(0) << ' ' << ab_aruco(1)  << ' ' << ab_aruco(2) << ' ' << ab_aruco(3) << ' ' << ab_aruco(4) << ' ' << ab_aruco(5) <<'\n';
        // cout << mb_aruco(0) << ' ' << mb_aruco(1)  << ' ' << mb_aruco(2) << ' ' << mb_aruco(3) << ' ' << mb_aruco(4) << ' ' << mb_aruco(5) <<'\n';

        // cout << pose_ee(0) << ' ' << pose_ee(1)  << ' ' << pose_ee(2) << ' ' << pose_ee(3) << ' ' << pose_ee(4) << ' ' << pose_ee(5) <<'\n';

        // cout << mobile_commandp[0] << ' ' << mobile_commandp[1] << ' ' << mobile_commandp[2] << '\n';

        // cout << "TarPos: " << arm_targetp[0]*180/PI << setw(8) << arm_targetp[1]*180/PI << setw(8) << arm_targetp[2]*180/PI << '\n';
        // cout << "TarXYZ: " << arm_traj_pose(0) << setw(8) << arm_traj_pose(1) << setw(8) << arm_traj_pose(2) << '\n';
        // cout << "TarRPY: " << arm_traj_pose_quat(0)*180/PI << setw(8) << arm_traj_pose_quat(1)*180/PI << setw(8) << arm_traj_pose_quat(2)*180/PI << '\n';

        // cout << "CurTor: " << dynamic_torque[0] << setw(8) << dynamic_torque[1] << setw(8) << dynamic_torque[2] << '\n';
        // cout << "CurTor: " << dynamic_torque[0] << setw(8) << dynamic_torque[1] << setw(8) << dynamic_torque[2] << setw(8) << dynamic_torque[3] << setw(8) << dynamic_torque[4] << setw(8) << dynamic_torque[5] << '\n';

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}