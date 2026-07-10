#include "dual_arm_function.cpp"

double dual_arm_cmd[9] = {0,};
int a = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dula_arm_command_publisher");
    ros::NodeHandle nh;

    ros::Publisher dual_arm_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/DualArmCmd_sim",100);

	std_msgs::Float32MultiArray dual_arm_command_msg;

	ros::Rate loop_rate(100);

    while(ros::ok())
    {
	cout << "Enter 0,1,2,3,4,5,6" << endl;
	cin >> a;
	if(a==0){
		cout << "Please enter the 9 joint angles for the dual arm. " << endl;
		cout << "Enter the target waist joint angle : " << endl;
		cin >> dual_arm_cmd[0];
		cout << "Enter the target left arm joint angle : " << endl;
		for(int i = 1; i < 5; i++){
			cin >> dual_arm_cmd[i];
		}
		cout << "Enter the target right arm joint angle : " << endl;
		for(int i = 5; i < 9; i++){
			cin >> dual_arm_cmd[i];
		}
	}

	else if(a==1){
		dual_arm_cmd[0] = 45;
		dual_arm_cmd[1] = 0;
		dual_arm_cmd[2] = 0;
		dual_arm_cmd[3] = 0;
		dual_arm_cmd[4] = 0;
		dual_arm_cmd[5] = 0;
		dual_arm_cmd[6] = 0;
		dual_arm_cmd[7] = 0;
		dual_arm_cmd[8] = 0;
	}

	else if(a==2){
		dual_arm_cmd[0] = 0;
		dual_arm_cmd[1] = -90;
		dual_arm_cmd[2] = 0;
		dual_arm_cmd[3] = 0;
		dual_arm_cmd[4] = 0;
		dual_arm_cmd[5] = -90;
		dual_arm_cmd[6] = 0;
		dual_arm_cmd[7] = 0;
		dual_arm_cmd[8] = 0;
	}

	else if(a==3){
		dual_arm_cmd[0] = 0;
		dual_arm_cmd[1] = 0;
		dual_arm_cmd[2] = 85;
		dual_arm_cmd[3] = 0;
		dual_arm_cmd[4] = 0;
		dual_arm_cmd[5] = 0;
		dual_arm_cmd[6] = -85;
		dual_arm_cmd[7] = 0;
		dual_arm_cmd[8] = 0;
	}

	else if(a==4){
		dual_arm_cmd[0] = 0;
		dual_arm_cmd[1] = -90;
		dual_arm_cmd[2] = 0;
		dual_arm_cmd[3] = 90;
		dual_arm_cmd[4] = -90;
		dual_arm_cmd[5] = -90;
		dual_arm_cmd[6] = 0;
		dual_arm_cmd[7] = -90;
		dual_arm_cmd[8] = -90;
	}

	else if(a==5){
		dual_arm_cmd[0] = 0;
		dual_arm_cmd[1] = 0;
		dual_arm_cmd[2] = 0;
		dual_arm_cmd[3] = 0;
		dual_arm_cmd[4] = -90;
		dual_arm_cmd[5] = 0;
		dual_arm_cmd[6] = 0;
		dual_arm_cmd[7] = 0;
		dual_arm_cmd[8] = -90;
	}

	else if(a==6){
		dual_arm_cmd[0] = 0;
		dual_arm_cmd[1] = 0;
		dual_arm_cmd[2] = 0;
		dual_arm_cmd[3] = 0;
		dual_arm_cmd[4] = 0;
		dual_arm_cmd[5] = 0;
		dual_arm_cmd[6] = 0;
		dual_arm_cmd[7] = 0;
		dual_arm_cmd[8] = 0;
	}

		dual_arm_command_msg.data.clear();
		for(int i = 0; i < 9; i++) {
			dual_arm_command_msg.data.push_back(dual_arm_cmd[i]*deg2rad);
		}
		dual_arm_cmd_pub.publish(dual_arm_command_msg);

        ros::spinOnce();
    }
	return 0;
}