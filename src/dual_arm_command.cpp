#include "dual_arm_function.cpp"

double dual_arm_cmd[DoF] = {0,};   // modeling용 관절각 (0:waist 1:head_yaw 2:head_pitch 3~6:L arm 7~10:R arm)
double ee_target[6] = {0,};      // simulation용 목표 EE 위치
int a = 0;

// 현재 어느 화면에 있는지 나타내는 상태
enum Screen {
    SCREEN_TOP,      // modeling / simulation / vision 선택
    SCREEN_MODELING, // 0~6 입력
    SCREEN_SPACE,    // joint / cartesian 선택
    SCREEN_TARGET,   // 0 입력 시 xyz 6개 입력
    SCREEN_VISION_TARGET // 0 입력 시 이송 목표 xyz 3개 입력 (물체 위치는 /aruco_ros/pose에서 자동 획득)
};

// back / b 입력인지 확인하는 함수
bool isBack(const string& s) {
    return (s == "back" || s == "b" || s == "B");
}

// q / Q / quit 입력인지 확인하는 함수
bool isQuit(const string& s) {
    return (s == "q" || s == "Q" || s == "quit");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_command_publisher");
    ros::NodeHandle nh;
    ros::Publisher dual_arm_cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("/dual_arm/DualArmCmd_sim", 100);
    std_msgs::Float32MultiArray dual_arm_command_msg;
    ros::Rate loop_rate(100);

    Screen screen = SCREEN_TOP;   // 시작은 최상위 화면
    int sim_mode_flag = 1;        // 1:joint 2:cartesian (SPACE에서 선택)
    bool quit_requested = false;  // q 입력 시 true로 바뀌어 루프 종료

    while (ros::ok() && !quit_requested)
    {
        bool do_publish = false;          // 이번 턴에 발행할지 여부 (back이면 false 유지)
        dual_arm_command_msg.data.clear();

        // ============================================================
        // 화면 1: 최상위 (modeling / simulation)
        // ============================================================
        if (screen == SCREEN_TOP) {
            string top_mode;
            cout << "\n[TOP] Select mode: modeling / simulation / vision  (or 'q' to quit)" << endl;
            cin >> top_mode;

            if (isQuit(top_mode)) {
                quit_requested = true;
            }
            else if (top_mode == "modeling") {
                screen = SCREEN_MODELING;
            }
            else if (top_mode == "simulation") {
                screen = SCREEN_SPACE;
            }
            else if (top_mode == "vision") {
                screen = SCREEN_VISION_TARGET;
            }
            else {
                cout << "Unknown input. Try again." << endl;
            }
            // 최상위에서는 발행하지 않고 화면만 전환
        }

        // ============================================================
        // 화면 2: modeling (0~6 입력, back으로 TOP 복귀)
        // ============================================================
        else if (screen == SCREEN_MODELING) {
            string in;
            cout << "\n[MODELING] Enter 0~6  (or 'back' to go up, 'q' to quit)" << endl;
            cin >> in;

            if (isQuit(in)) {
                quit_requested = true;
            }
            else if (isBack(in)) {
                screen = SCREEN_TOP;     // 이전 화면으로, 발행 안 함
            }
            else {
                a = atoi(in.c_str());    // 숫자로 변환

                if (a == 0) {
                    cout << "Enter waist angle: " << endl;
                    cin >> dual_arm_cmd[0];

                    cout << "Enter head angles(2, yaw pitch): " << endl;
                    for (int i = 1; i < 3; i++) {
                        cin >> dual_arm_cmd[i];
                    }

                    cout << "Enter left arm angles(4): " << endl;
                    for (int i = 3; i < 7; i++) {
                        cin >> dual_arm_cmd[i];
                    }

                    cout << "Enter right arm angles(4): " << endl;
                    for (int i = 7; i < 11; i++) {
                        cin >> dual_arm_cmd[i];
                    }
                    do_publish = true;
                }
                else if (a == 1) {
                    double t[DoF] = {45, 0,0, 0, 0, 0, 0, 0, 0, 0, 0};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else if (a == 2) {
                    double t[DoF] = {0, 0,0, -90, 0, 0, 0, -90, 0, 0, 0};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else if (a == 3) {
                    double t[DoF] = {0, 0,0, 0, 85, 0, 0, 0, -85, 0, 0};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else if (a == 4) {
                    double t[DoF] = {0, 0,0, -90, 0, 90, -90, -90, 0, -90, -90};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else if (a == 5) {
                    double t[DoF] = {0, 0,0, 0, 0, -90, 0, 0, 0, 0, -90};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else if (a == 6) {
                    double t[DoF] = {0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0};
                    for (int i = 0; i < DoF; i++) {
                        dual_arm_cmd[i] = t[i];
                    }
                    do_publish = true;
                }
                else {
                    cout << "Invalid number (0~6)." << endl;
                }
            }

            // 값 세팅이 끝난 뒤, 공통으로 메시지 채움 (modeling 전용 포맷: 모드 플래그 0 + 관절각 DoF개)
            if (do_publish) {
                dual_arm_command_msg.data.push_back(0.0f);   // 모드 플래그 0 (modeling)
                for (int i = 0; i < DoF; i++) {
                    dual_arm_command_msg.data.push_back(dual_arm_cmd[i] * deg2rad);
                }
            }
        }

        // ============================================================
        // 화면 3: space 선택 (joint / cartesian, back으로 TOP 복귀)
        // ============================================================
        else if (screen == SCREEN_SPACE) {
            string sp;
            cout << "\n[SIMULATION] Select space: joint / cartesian  (or 'back' to go up, 'q' to quit)" << endl;
            cin >> sp;

            if (isQuit(sp)) {
                quit_requested = true;
            }
            else if (isBack(sp)) {
                screen = SCREEN_TOP;     // 이전 화면으로
            }
            else if (sp == "joint") {
                sim_mode_flag = 1;
                screen = SCREEN_TARGET;
            }
            else if (sp == "cartesian") {
                sim_mode_flag = 2;
                screen = SCREEN_TARGET;
            }
            else {
                cout << "Unknown input. Try again." << endl;
            }
            // space 화면에서는 발행 안 함
        }

        // ============================================================
        // 화면 4: target 입력 (0 입력 시 xyz 6개, back으로 SPACE 복귀)
        // ============================================================
        else if (screen == SCREEN_TARGET) {
            string in;
            cout << "\n[" << (sim_mode_flag == 1 ? "JOINT" : "CARTESIAN")
                 << "] Enter 0 to input target  (or 'back' to go up, 'q' to quit)" << endl;
            cin >> in;

            if (isQuit(in)) {
                quit_requested = true;
            }
            else if (isBack(in)) {
                screen = SCREEN_SPACE;   // 이전 화면(space)으로
            }
            else {
                int sel = atoi(in.c_str());
                if (sel == 0) {
                    cout << "Enter target EE position [Lx Ly Lz Rx Ry Rz] (meter):" << endl;
                    for (int i = 0; i < 6; i++) {
                        cin >> ee_target[i];
                    }
                    do_publish = true;
                }
                else {
                    cout << "Press 0 to input target, or 'back'." << endl;
                }
            }

            // 값 세팅이 끝난 뒤, 공통으로 메시지 채움 (target 전용 포맷: sim_mode_flag + EE 위치 6개)
            if (do_publish) {
                dual_arm_command_msg.data.push_back((float)sim_mode_flag);  // 1 또는 2
                for (int i = 0; i < 6; i++) {
                    dual_arm_command_msg.data.push_back((float)ee_target[i]);
                }
            }
        }

        // ============================================================
        // 화면 5: vision target 입력 (0 입력 시 이송 목표 xyz 3개, back으로 TOP 복귀)
        // 물체 위치 자체는 이 노드가 아니라 dual_arm_main이 /aruco_ros/pose를 직접 구독해서 얻음.
        // ============================================================
        else if (screen == SCREEN_VISION_TARGET) {
            string in;
            cout << "\n[VISION] Enter 0 to input transport target position "
                 << "(or 'back' to go up, 'q' to quit)" << endl;
            cin >> in;

            if (isQuit(in)) {
                quit_requested = true;
            }
            else if (isBack(in)) {
                screen = SCREEN_TOP;
            }
            else {
                int sel = atoi(in.c_str());
                if (sel == 0) {
                    double transport_target[3];
                    cout << "Enter transport target position [x y z] (meter, world frame):" << endl;
                    for (int i = 0; i < 3; i++) {
                        cin >> transport_target[i];
                    }
                    dual_arm_command_msg.data.push_back(3.0f);  // 모드 플래그 3 (vision pick)
                    for (int i = 0; i < 3; i++) {
                        dual_arm_command_msg.data.push_back((float)transport_target[i]);
                    }
                    do_publish = true;
                }
                else {
                    cout << "Press 0 to input target, or 'back'." << endl;
                }
            }
        }

        // ============================================================
        // 발행 (do_publish 가 true 일 때만 = back/화면전환이면 발행 안 됨)
        // ============================================================
        if (do_publish) {
            dual_arm_cmd_pub.publish(dual_arm_command_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << "\nShutting down dual_arm_command_publisher..." << endl;
    return 0;
}