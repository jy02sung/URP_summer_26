# Dual_Arm_ROS_Simulation
Dual Arm Robot
* Author : Seungjun Lee
* E-mail : sjun0107@g.skku.edu
## System Requirements :
* Ubuntu 20.04 + ROS Noetic
## Dependencies :
```bash
sudo apt update
```
```bash
sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
```bash
sudo apt install robotpkg-py38-pinocchio
```
## Build :
```bash
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) -l$(nproc)
```
## Launch and Run gazebo simulation & Rviz :
* Step 1: Open two terminal windows
* Step 2 (Terminal 1): Launch Gazebo :
```bash
roslaunch dual_arm gazebo.launch
```
* Step 3 (Terminal 2): Execute the command :
```bash
rosrun dual_arm dual_arm_command
```
<img width="1921" height="1079" alt="Screenshot from 2026-06-26 18-21-25" src="https://github.com/user-attachments/assets/df6e43b1-f1ba-4e93-a99f-41acb4c90c4a" />


* Launch RViz :
```bash
roslaunch dual_arm rviz.launch
```
<img width="1921" height="1078" alt="Screenshot from 2026-06-26 18-22-40" src="https://github.com/user-attachments/assets/6b079a54-f01a-4db3-8e9c-2d9d7d60de77" />

