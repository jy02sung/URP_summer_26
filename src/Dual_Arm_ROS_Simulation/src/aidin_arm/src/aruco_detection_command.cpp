#include "arm_function.cpp"

int aruco_marker;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_detection_command_publisher");
    ros::NodeHandle nh;

    ros::Publisher aruco_detection_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/aidin_arm/ArucoDetectionCmd_sim",100);

	std_msgs::Float32MultiArray aruco_detection_command_msg;

	ros::Rate loop_rate(100);

    while(ros::ok())
    {
		cout << "Enter the target aruco marker : " << '\n';
		cin >> aruco_marker;

		aruco_detection_command_msg.data.clear();
		aruco_detection_command_msg.data.push_back(aruco_marker);
		aruco_detection_cmd_pub.publish(aruco_detection_command_msg);

        ros::spinOnce();
    }
	return 0;
}