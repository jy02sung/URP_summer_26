#include "dual_arm_function.cpp"
#include <pinocchio/parsers/urdf.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>

#include <cmath>
#include <array>
#include <iostream>

int main(int argc, char** argv) {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(ros::package::getPath("dual_arm") + "/urdf/dual_arm.urdf", model);
    pinocchio::Data data(model);
    const auto left_frame = model.getFrameId("L_wrist_ik_frame");
    const auto right_frame = model.getFrameId("R_wrist_ik_frame");
    if (model.nq != DoF || left_frame == model.nframes || right_frame == model.nframes) return 2;

    if (argc > 1 && std::string(argv[1]) == "--measure-controller") {
        ros::init(argc, argv, "mission4_controller_measure");
        ros::NodeHandle nh;
        const auto state = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/dual_arm/joint_states", nh, ros::Duration(5.0));
        if (!state) return 3;
        const std::array<std::string, DoF> names = {{
            "Waist_joint", "Head_yaw_joint", "Head_pitch_joint",
            "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
            "L_elbow_joint", "L_wrist_yaw_joint", "L_wrist_pitch_joint",
            "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
            "R_elbow_joint", "R_wrist_yaw_joint", "R_wrist_pitch_joint"}};
        const std::array<int, DoF> controller_to_pin = {{0,3,4,5,6,7,8,9,10,11,12,13,14,1,2}};
        VectorXd actual = VectorXd::Zero(DoF), command = VectorXd::Zero(DoF);
        for (std::size_t j=0;j<state->name.size();++j)
            for(int i=0;i<DoF;++i) if(state->name[j]==names[i]) actual(i)=state->position[j];
        for (int c=0;c<DoF;++c) {
            const auto msg=ros::topic::waitForMessage<control_msgs::JointControllerState>(
                "/dual_arm/joint"+std::to_string(c+1)+"_position_controller/state",nh,ros::Duration(2.0));
            if(!msg) return 4;
            command(controller_to_pin[c])=msg->set_point;
        }
        pinocchio::forwardKinematics(model,data,command); pinocchio::updateFramePlacements(model,data);
        const Vector3d desired_l=data.oMf[left_frame].translation(), desired_r=data.oMf[right_frame].translation();
        pinocchio::forwardKinematics(model,data,actual); pinocchio::updateFramePlacements(model,data);
        std::cout<<"runtime_left_error="<<(desired_l-data.oMf[left_frame].translation()).norm()
                 <<" runtime_right_error="<<(desired_r-data.oMf[right_frame].translation()).norm()
                 <<" torso_command_error="<<(command.head<3>()-actual.head<3>()).norm()<<'\n';
        return 0;
    }

    VectorXd seed = VectorXd::Zero(DoF);
    const bool pre_squeeze = argc > 1 && std::string(argv[1]) == "--pre-squeeze";
    const bool publish_demo = (argc > 1 && std::string(argv[1]) == "--publish") || pre_squeeze;
    if (publish_demo) {
        ros::init(argc, argv, "mission4_ik_demo");
        ros::NodeHandle nh;
        const auto state = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/dual_arm/joint_states", nh, ros::Duration(5.0));
        if (!state) return 3;
        const std::array<std::string, DoF> names = {{
            "Waist_joint", "Head_yaw_joint", "Head_pitch_joint",
            "L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint",
            "L_elbow_joint", "L_wrist_yaw_joint", "L_wrist_pitch_joint",
            "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint",
            "R_elbow_joint", "R_wrist_yaw_joint", "R_wrist_pitch_joint"}};
        for (std::size_t j = 0; j < state->name.size(); ++j)
            for (int i = 0; i < DoF; ++i)
                if (state->name[j] == names[i]) seed(i) = state->position[j];
    } else {
        seed(0) = 0.12;   // waist/head must remain exactly unchanged by arm IK
        seed(1) = -0.08;
        seed(2) = 0.18;
    }
    pinocchio::forwardKinematics(model, data, seed);
    pinocchio::updateFramePlacements(model, data);
    const Vector3d start_left = data.oMf[left_frame].translation();
    const Vector3d start_right = data.oMf[right_frame].translation();

    // Mission 5 first gate: stop 20mm outside the 150mm box surfaces without contact.
    // Otherwise retain the small bilateral Mission 4 regression displacement.
    const Vector3d target_left = pre_squeeze ? Vector3d(0.45, 0.095, 1.225)
                                               : start_left + Vector3d(0.015, -0.010, 0.010);
    const Vector3d target_right = pre_squeeze ? Vector3d(0.45, -0.095, 1.225)
                                                : start_right + Vector3d(0.015, 0.010, 0.010);
    VectorXd result;
    dualarm.SolveIK_Position(model, data, left_frame, right_frame,
                             target_left, target_right, seed, result);

    pinocchio::forwardKinematics(model, data, result);
    pinocchio::updateFramePlacements(model, data);
    const double left_error = (target_left - data.oMf[left_frame].translation()).norm();
    const double right_error = (target_right - data.oMf[right_frame].translation()).norm();
    const double torso_error = (result.head<3>() - seed.head<3>()).cwiseAbs().maxCoeff();
    double wrist_max = 0.0;
    for (const int i : {7, 8, 13, 14}) wrist_max = std::max(wrist_max, std::abs(result(i)));

    std::cout << "left_error=" << left_error << " right_error=" << right_error
              << " torso_error=" << torso_error << " wrist_max=" << wrist_max << '\n';
    std::cout << "result=" << result.transpose() << '\n';

    const bool pass = left_error < 0.003 && right_error < 0.003 &&
                      torso_error < 1e-12 && wrist_max <= 0.3500001 && result.allFinite();
    std::cout << (pass ? "MISSION4_IK_PASS" : "MISSION4_IK_FAIL") << '\n';
    if (pass && publish_demo) {
        ros::NodeHandle nh;
        if (pre_squeeze) {
            ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(
                "/dual_arm/gravity_pd_target", 1, true);
            ros::WallDuration(0.5).sleep();
            ros::WallRate rate(100);
            // 25-second quintic blend from the measured current state.  The gravity PD
            // controller applies its own torque slew limit, so both position and effort
            // commands remain continuous.
            for (int tick = 0; ros::ok() && tick < 3000; ++tick) {
                const double u = std::min(1.0, tick / 2499.0);
                const double blend = 10*std::pow(u, 3) - 15*std::pow(u, 4) + 6*std::pow(u, 5);
                const VectorXd command = seed + blend * (result - seed);
                std_msgs::Float64MultiArray message;
                message.data.resize(DoF);
                for (int i = 0; i < DoF; ++i) message.data[i] = command(i);
                pub.publish(message);
                rate.sleep();
            }
            return 0;
        }
        const std::array<int, DoF> controller_to_pin = {{
            0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 1, 2}};
        std::array<ros::Publisher, DoF> pubs;
        for (int i = 0; i < DoF; ++i)
            pubs[i] = nh.advertise<std_msgs::Float64>(
                "/dual_arm/joint" + std::to_string(i + 1) + "_position_controller/command", 1);
        ros::WallDuration(0.5).sleep();
        ros::Rate rate(50);
        // Fifteen-second quintic blend keeps the X-Z wrist transient comfortably bounded.
        // Hold the final target for another five seconds for settling measurements.
        for (int tick = 0; ros::ok() && tick < 1000; ++tick) {
            const double u = std::min(1.0, tick / 749.0);
            const double blend = 10*std::pow(u, 3) - 15*std::pow(u, 4) + 6*std::pow(u, 5);
            const VectorXd command = seed + blend * (result - seed);
            for (int i = 0; i < DoF; ++i) {
                std_msgs::Float64 message;
                message.data = command(controller_to_pin[i]);
                pubs[i].publish(message);
            }
            rate.sleep();
        }
    }
    return pass ? 0 : 1;
}
