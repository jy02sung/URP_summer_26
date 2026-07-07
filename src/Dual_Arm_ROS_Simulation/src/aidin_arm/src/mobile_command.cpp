#include "arm_function.cpp"

double xy_cmd[2] = {0, };

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_command_publisher");
    ros::NodeHandle nh;

    ros::Publisher mobile_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/aidin_arm/MobileCmd_sim",100);
	// ros::Publisher mobile_cmd_mode_pub = nh.advertise<std_msgs::Float64>("/aidin_arm/MobileCmdMode_sim",100);

	std_msgs::Float32MultiArray mobile_command_msg;
	// std_msgs::Float64 mobile_cmd_mode_msg;

	ros::Rate loop_rate(100);

	// cout << "control start click 1" << '\n';
	// cin >> mobile_cmd_mode;

    while(ros::ok())
    {
		cout << "Enter the target mobile position : " << '\n';
		cin >> xy_cmd[0] >> xy_cmd[1];

		// mobile_cmd_mode_msg.data = mobile_cmd_mode;
		// mobile_cmd_mode_pub.publish(mobile_cmd_mode_msg);

		mobile_command_msg.data.clear();
		for(int i = 0; i < 2; i++) {
			mobile_command_msg.data.push_back(xy_cmd[i]);
		}
		mobile_cmd_pub.publish(mobile_command_msg);

        ros::spinOnce();
    }
	return 0;
}