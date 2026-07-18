#!/usr/bin/env python3

import time

import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from gazebo_msgs.srv import SetModelConfiguration
from std_srvs.srv import Empty


EXPECTED_CONTROLLERS = {
    "joint_state_controller",
    *{"joint{}_position_controller".format(i) for i in range(1, 16)},
}

STARTUP_JOINT_NAMES = [
    "Waist_joint", "L_shoulder_pitch_joint", "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint", "L_elbow_joint", "L_wrist_yaw_joint",
    "L_wrist_pitch_joint", "R_shoulder_pitch_joint", "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint", "R_elbow_joint", "R_wrist_yaw_joint",
    "R_wrist_pitch_joint", "Head_yaw_joint", "Head_pitch_joint",
]
STARTUP_JOINT_POSITIONS = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
]


def main():
    rospy.init_node("unpause_when_controllers_ready")
    rospy.wait_for_service("/dual_arm/controller_manager/list_controllers")
    list_controllers = rospy.ServiceProxy(
        "/dual_arm/controller_manager/list_controllers", ListControllers
    )

    while not rospy.is_shutdown():
        states = {controller.name: controller.state for controller in list_controllers().controller}
        # A paused Gazebo cannot advance controller_manager's switch request to
        # "running" because that transition happens in the physics update loop.
        # Waiting until every controller is present removes the long uncontrolled
        # spawn interval; the first unpaused update completes the switch together.
        if EXPECTED_CONTROLLERS.issubset(states):
            break
        time.sleep(0.05)  # Gazebo is paused, so wall time must be used here.

    if rospy.is_shutdown():
        return

    # spawn_model의 -J 옵션은 gazebo_ros_control 플러그인이 늦게 로드되는 이 모델에서
    # 성공을 보고하고도 실제 관절값을 0으로 남긴다. 모든 컨트롤러가 로드된 뒤, 물리가
    # 아직 멈춘 상태에서 시작 자세를 다시 설정해야 첫 tick의 충돌 임펄스를 피할 수 있다.
    rospy.wait_for_service("/gazebo/set_model_configuration")
    set_configuration = rospy.ServiceProxy(
        "/gazebo/set_model_configuration", SetModelConfiguration
    )
    configured = set_configuration(
        model_name="dual_arm",
        urdf_param_name="robot_description",
        joint_names=STARTUP_JOINT_NAMES,
        joint_positions=STARTUP_JOINT_POSITIONS,
    )
    if not configured.success:
        raise RuntimeError("Failed to apply the collision-free startup pose: {}".format(
            configured.status_message
        ))

    rospy.wait_for_service("/gazebo/unpause_physics")
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    rospy.wait_for_service("/dual_arm/controller_manager/switch_controller")
    switch_controllers = rospy.ServiceProxy(
        "/dual_arm/controller_manager/switch_controller", SwitchController
    )
    response = switch_controllers(
        start_controllers=sorted(EXPECTED_CONTROLLERS),
        stop_controllers=[],
        strictness=SwitchControllerRequest.STRICT,
        start_asap=True,
        timeout=2.0,
    )
    if not response.ok:
        raise RuntimeError("Failed to start the complete effort-controller set")
    rospy.loginfo("All startup hold controllers started; Gazebo physics is running.")


if __name__ == "__main__":
    main()
