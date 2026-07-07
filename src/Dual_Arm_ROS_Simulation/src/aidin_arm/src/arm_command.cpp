#include "arm_function.cpp"

double armjointp_sim[DoF] = {0, };
double armjointp_cmd[DoF] = {0, };

void msgCallbackArmJointPos_sim(const sensor_msgs::JointState::ConstPtr& msg)
{
	for (int i = 0; i < DoF; i++)
	{
		armjointp_sim[i] = msg->position[i];
	}
}

void signal_handler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

void choose_mode(int *mode, int size, string prompt) {
	cout << prompt << '\n';
	cin >> *mode;
	while(!(*mode == -1 || *mode >= 1 && *mode <= size)) {
		cout << "Enter a value between 1 and " << size <<  ".\n";
		cin >> *mode;
	}
	if(*mode == -1)
		exit(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_command_publisher");
    ros::NodeHandle nh;

	ros::Publisher arm_command_pub = nh.advertise<std_msgs::Float32MultiArray>("/aidin_arm/ArmCmd_sim",100);
	ros::Publisher arm_cmd_mode_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/ArmCmdMode_sim",100);
	ros::Subscriber sub_joint_angle = nh.subscribe("/aidin_arm/joint_states", 100, msgCallbackArmJointPos_sim);
    // ros::Subscriber sub_bodypose = nh.subscribe("/aidin81/BodyPose_sim", 100, msgCallbackBodyPose_sim);
	std_msgs::Float32MultiArray joint_command_msg;
	std_msgs::Float64 arm_cmd_mode_msg;

	const double d1 = 0.108;
	const double a2 = 0.35;
	const double a3 = 0.1;
	const double d4 = 0.4;
	const double d6 = 0.1;

    int op_mode = 1;

	ros::Rate loop_rate(100);

	signal(SIGINT, signal_handler);

	cout << "Choose control space (1: joint / 2: Cartesian)" << '\n';
	cin >> arm_cmd_mode;
	if( arm_cmd_mode != 1 && arm_cmd_mode != 2 ) {
		arm_cmd_mode = 1; // default joint space
	}

    while (ros::ok())
    {
		choose_mode(&op_mode, 4, "Choose control mode (1: home / 2: joint position / 3: EE pose / 4: Current pose) : ");
		// Homing (홈포지션으로 초기화)
        if(op_mode == 1) 
        {
            int cmd1 = 0;
			// cout << "Choose pose (1: Body initial / 2: Straight / 3: Down / 4: Turn) : " << '\n';
			choose_mode(&cmd1, 6, "Choose pose (0: up / 1: body initial / 2: Straight / 3: Down / 4: right Turn / 5: left Turn) : ");

            if(cmd1 == 0){ // upppp
				armjointp_cmd[0] = 0;
				armjointp_cmd[1] = 0;
				armjointp_cmd[2] = 0;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = 0;
				armjointp_cmd[5] = 0;
			}
			else if(cmd1 == 1){ // body initial (-0.137 0 0.158 0 60 0)
				armjointp_cmd[0] = 0;
				armjointp_cmd[1] = -PI/2;
				armjointp_cmd[2] = PI;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = 0;
				armjointp_cmd[5] = 0;
			}
			else if(cmd1 == 2){ // straight
				armjointp_cmd[0] = 0;
				armjointp_cmd[1] = PI/3;
				armjointp_cmd[2] = PI/3;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = -PI/6;
				armjointp_cmd[5] = 0;
			}
			else if(cmd1 == 3){ // down
				armjointp_cmd[0] = 0;
				armjointp_cmd[1] = PI/2;
				armjointp_cmd[2] = PI/2;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = 0;
				armjointp_cmd[5] = 0;
			}
			else if(cmd1 == 4){ // straight turn
				armjointp_cmd[0] = 2*PI;
				armjointp_cmd[1] = PI/3;
				armjointp_cmd[2] = PI/3;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = -PI/6;
				armjointp_cmd[5] = 0;
			}
			else if(cmd1 == 5){ // left
				armjointp_cmd[0] = PI/2;
				armjointp_cmd[1] = PI/3;
				armjointp_cmd[2] = PI/3;
				armjointp_cmd[3] = 0;
				armjointp_cmd[4] = -PI/6;
				armjointp_cmd[5] = 0;
			}

			armjointp_cmd[1] -= PI/2; // DH parameter changed
        }

        else if (op_mode == 2) // Joint position
        {
			cout << "Enter the target joint position[degree] : " << '\n';
            cin >> armjointp_cmd[0] >> armjointp_cmd[1] >> armjointp_cmd[2] >> armjointp_cmd[3] >> armjointp_cmd[4] >> armjointp_cmd[5];
            if (abs(armjointp_cmd[0]) > 360 or abs(armjointp_cmd[1]) > 360 or abs(armjointp_cmd[2]) > 360)
            {
                cout << "Out of Joint Configuration!\n\n";
                continue;
            }
            for (int i = 0; i < DoF; i++)
            {
                armjointp_cmd[i] *= DEG2RAD;
            }
        }

        else if (op_mode == 3) // EE pose
        {
            double x, y, z, roll, pitch, yaw;
            VectorXd pose_ini = VectorXd::Zero(6);

            cout << "Enter the target position : " << '\n';
            cin >> x >> y >> z >> roll >> pitch >> yaw;
			double _ee_pos = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
			double _ee_pos_limit = d1 + a2 + sqrt(pow(a3, 2) + pow(d4, 2)) + d6;
			if(_ee_pos > _ee_pos_limit)
            {
                cout << "Out of Configuration!" << '\n';
                continue;
            }
            pose_cmd << x, y, z, roll * DEG2RAD, pitch * DEG2RAD, yaw * DEG2RAD;
			arm.inverseKinematics(pose_cmd, armjointp_sim, armjointp_cmd);
        }
		
		// Current pose (debug용)
		else if(op_mode == 4) {  
			cout << "Current pose: This feature is still under development.\n";
		}
		else {
			cout << "try again!\n\n";
			continue;
		}

        arm_cmd_mode_msg.data = arm_cmd_mode;
		arm_cmd_mode_pub.publish(arm_cmd_mode_msg);

		joint_command_msg.data.clear();
        for (int i = 0; i < DoF; i++)
        {
            joint_command_msg.data.push_back(armjointp_cmd[i]);
        }
        arm_command_pub.publish(joint_command_msg);

        ros::spinOnce();
    }
	return 0;
}