#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <cmath>

using namespace std;

// 11 DoF 명령을 담을 수 있도록 배열 크기 확장
double dual_arm_cmd[11] = {0,};
int a = 0;
const double deg2rad = M_PI / 180;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dual_arm_command_publisher");
    ros::NodeHandle nh;

    ros::Publisher dual_arm_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/DualArmCmd_sim",100);
    ros::Publisher gain_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/PDGainCmd_sim", 10);
    
    ros::Publisher cart_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/CartesianCmd_sim", 100);
    ros::Publisher joint_ik_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/JointIKCmd_sim", 100);

	std_msgs::Float32MultiArray dual_arm_command_msg;
	ros::Rate loop_rate(100);

    while(ros::ok())
    {
        // 메뉴 문구 업데이트 (0번 직선, 9번 곡선)
        cout << "\n=============================================" << endl;
        cout << " 8: Joint Angle Move    |  1~6: Presets" << endl;
        cout << " 7: Change PD Gains" << endl;
        cout << " 0: [Cartesian] XYZ Straight Line Move" << endl;
        cout << " 9: [Joint IK]  XYZ Curved Line Move" << endl;
        cout << "=============================================" << endl;
        cout << "Enter command: ";
        cin >> a;

        if (a == 7) {
            float p_in, d_in;
            cout << "Enter P gain for all joints: "; cin >> p_in;
            cout << "Enter D gain for all joints: "; cin >> d_in;
            
            std_msgs::Float32MultiArray gain_msg;
            gain_msg.data.push_back(p_in);
            gain_msg.data.push_back(d_in);
            gain_cmd_pub.publish(gain_msg);
            
            cout << ">>> Published new PD Gains: P=" << p_in << ", D=" << d_in << endl;
            ros::spinOnce();
            continue; 
        }
        else if(a == 0 || a == 9){
            float xl, yl, zl, xr, yr, zr;
            cout << "Enter LEFT arm target (X Y Z) ex) 0.3 0.3 0.5 : ";
            cin >> xl >> yl >> zl;
            cout << "Enter RIGHT arm target (X Y Z) ex) 0.3 -0.3 0.5 : ";
            cin >> xr >> yr >> zr;
            
            std_msgs::Float32MultiArray xyz_msg;
            xyz_msg.data.push_back(xl); xyz_msg.data.push_back(yl); xyz_msg.data.push_back(zl);
            xyz_msg.data.push_back(xr); xyz_msg.data.push_back(yr); xyz_msg.data.push_back(zr);
            
            if (a == 0) {
                cart_cmd_pub.publish(xyz_msg);
                cout << ">>> [Mode 0] Cartesian Straight-Line Targets Sent!" << endl;
            } else {
                joint_ik_cmd_pub.publish(xyz_msg);
                cout << ">>> [Mode 9] Joint-Space Curved-Line Targets Sent!" << endl;
            }
            
            ros::spinOnce();
            continue; 
        }
        else if(a==8){
            cout << "Please enter the 11 joint angles for the dual arm. " << endl;
            cout << "Enter the target waist joint angle : " << endl;
            cin >> dual_arm_cmd[0];
            cout << "Enter the target left arm joint angle (4 joints) : " << endl;
            for(int i = 1; i < 5; i++){
                cin >> dual_arm_cmd[i];
            }
            cout << "Enter the target right arm joint angle (4 joints) : " << endl;
            for(int i = 5; i < 9; i++){
                cin >> dual_arm_cmd[i];
            }
            cout << "Enter the target head joint angle (Yaw, Pitch) : " << endl;
            for(int i = 9; i < 11; i++){
                cin >> dual_arm_cmd[i];
            }
        }
        else if(a==1){
            dual_arm_cmd[0] = 45;
            for(int i=1; i<11; i++) dual_arm_cmd[i] = 0;
        }
        else if(a==2){
            for(int i=0; i<11; i++) dual_arm_cmd[i] = 0;
            dual_arm_cmd[1] = -90;
            dual_arm_cmd[5] = -90;
        }
        else if(a==3){
            for(int i=0; i<11; i++) dual_arm_cmd[i] = 0;
            dual_arm_cmd[2] = 85;
            dual_arm_cmd[6] = -85;
        }
        else if(a==4){
            dual_arm_cmd[0] = 0;
            dual_arm_cmd[1] = -90; dual_arm_cmd[2] = 0; dual_arm_cmd[3] = 90; dual_arm_cmd[4] = -90;
            dual_arm_cmd[5] = -90; dual_arm_cmd[6] = 0; dual_arm_cmd[7] = -90; dual_arm_cmd[8] = -90;
            dual_arm_cmd[9] = 0; dual_arm_cmd[10] = 0;
        }
        else if(a==5){
            for(int i=0; i<11; i++) dual_arm_cmd[i] = 0;
            dual_arm_cmd[4] = -90;
            dual_arm_cmd[8] = -90;
        }
        else if(a==6){
            for(int i=0; i<11; i++) dual_arm_cmd[i] = 0;
        }

        dual_arm_command_msg.data.clear();
        for(int i = 0; i < 11; i++) {
            dual_arm_command_msg.data.push_back(dual_arm_cmd[i]*deg2rad);
        }
        dual_arm_cmd_pub.publish(dual_arm_command_msg);

        ros::spinOnce();
    }
	return 0;
}