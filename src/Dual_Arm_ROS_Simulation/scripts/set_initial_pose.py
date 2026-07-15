#!/usr/bin/env python3

import time

import rospy
from gazebo_msgs.srv import GetJointProperties, SetModelConfiguration


def main():
    rospy.init_node("dual_arm_initial_pose_setter")

    model_name = rospy.get_param("~model_name", "dual_arm")
    urdf_param = rospy.get_param("~urdf_param_name", "robot_description")
    settle_delay = rospy.get_param("~settle_delay", 2.0)
    controller_manager_service = rospy.get_param(
        "~controller_manager_service",
        "/dual_arm/controller_manager/list_controllers",
    )
    controller_wait = rospy.get_param("~controller_wait", 8.0)
    max_attempts = int(rospy.get_param("~max_attempts", 5))
    retry_delay = rospy.get_param("~retry_delay", 1.0)

    joint_names = [
        "Waist_joint",
        "L_shoulder_pitch_joint",
        "L_shoulder_roll_joint",
        "L_shoulder_yaw_joint",
        "L_elbow_joint",
        "L_wrist_yaw_joint",
        "R_shoulder_pitch_joint",
        "R_shoulder_roll_joint",
        "R_shoulder_yaw_joint",
        "R_elbow_joint",
        "R_wrist_yaw_joint",
        "Head_yaw_joint",
        "Head_pitch_joint",
    ]
    joint_positions = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    if all(abs(position) < 1e-12 for position in joint_positions):
        rospy.loginfo("Initial pose is standing zero pose; skipping set_model_configuration.")
        return

    rospy.wait_for_service("/gazebo/set_model_configuration")
    try:
        rospy.wait_for_service(controller_manager_service, timeout=controller_wait)
    except rospy.ROSException:
        rospy.logwarn(
            "Controller manager service %s was not ready after %.1fs; continuing",
            controller_manager_service,
            controller_wait,
        )

    time.sleep(settle_delay)

    set_model_configuration = rospy.ServiceProxy(
        "/gazebo/set_model_configuration", SetModelConfiguration
    )
    get_joint_properties = rospy.ServiceProxy(
        "/gazebo/get_joint_properties", GetJointProperties
    )

    for attempt in range(1, max_attempts + 1):
        response = set_model_configuration(
            model_name=model_name,
            urdf_param_name=urdf_param,
            joint_names=joint_names,
            joint_positions=joint_positions,
        )

        if not response.success:
            rospy.logwarn(
                "Initial pose attempt %d failed: %s", attempt, response.status_message
            )
        else:
            shoulder = get_joint_properties("L_shoulder_pitch_joint")
            elbow = get_joint_properties("L_elbow_joint")
            if (
                shoulder.success
                and elbow.success
                and abs(shoulder.position[0]) < 1e-6
                and abs(elbow.position[0]) < 1e-6
            ):
                rospy.loginfo("Initial pose applied to %s", model_name)
                return

            rospy.logwarn(
                "Initial pose attempt %d did not stick yet "
                "(shoulder=%.6f, elbow=%.6f)",
                attempt,
                shoulder.position[0] if shoulder.position else float("nan"),
                elbow.position[0] if elbow.position else float("nan"),
            )

        time.sleep(retry_delay)

    rospy.logerr("Failed to apply initial pose to %s after %d attempts", model_name, max_attempts)
    raise SystemExit(1)


if __name__ == "__main__":
    main()
