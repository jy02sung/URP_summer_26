# Vision-based_Mobile_Manipulator_ROS_Simulation
Vision-based Mobile Manipulator Robot
* Author : Seungjun Lee
* E-mail : sjun0107@g.skku.edu
## System Requirements :
* Ubuntu 20.04 + ROS Noetic
## Dependencies :
* Eigen version 3.4.0 recommand
```bash
sudo apt update
```
```bash
sudo apt-get install ros-noetic-controller-manager ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-position-controllers ros-noetic-robot-controllers ros-noetic-robot-state-publisher ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```
## Build :
```bash
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc) -l$(nproc)
```
## Launch and Run gazebo simulation & Rviz :
* Step 1: Open three terminal windows
* Step 2 (Terminal 1): Launch Gazebo :
```bash
roslaunch aidin_arm gazebo.launch
```
<img width="800" height="450" alt="image" src="https://github.com/user-attachments/assets/9289914d-344e-4361-98c8-44290bc9c79b" />

* Step 3 (Terminal 2): Launch Rviz :
```bash
roslaunch aidin_arm detector.launch
```
<img width="800" height="450" alt="아르코1" src="https://github.com/user-attachments/assets/3cbd4bfc-ea87-4020-a5f4-42db248f7941" />
<img width="800" height="450" alt="아르코2" src="https://github.com/user-attachments/assets/ba740c8b-eccb-4f20-a944-0ed2ff907fab" />

* Step 4 (Terminal 3): Execute the command :
```bash
rosrun aidin_arm aruco_detection_command
```
* Input 1 : Cartesian Path Planning
<img width="800" height="450" alt="카테시안" src="https://github.com/user-attachments/assets/6ec9df7b-38da-4096-84c8-60cf181ecd18" />

* Input 2 : Joint Path Planning
<img width="800" height="450" alt="조인트" src="https://github.com/user-attachments/assets/bd78b5ac-bce8-448f-9d85-66be19aced5d" />




