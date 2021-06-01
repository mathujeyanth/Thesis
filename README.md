# Autonomous Construction
A demonstration of reinforcement learning for high precision assemblies in Unity, where an agent is trained to assemble LEGO bricks.

![](docs/Images/AssemblyAgent.gif)
<!--<img src="./docs/Images/AssemblyAgent.gif" height=200px/>-->

## Files

* **Gripper** - the project containing the unity environment with the trained agent.
* **Thesis** - project containing a base framework allowing to control a UR5e through ROS.
* **ROS** - ROS packages allowing us to perform motion planning on the UR5e in Unity with MoveIt.

<!--## Unity Environment


## ROS Environment
```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt install python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf* ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher ros-noetic-industrial-robot-status-interface ros-noetic-industrial-msgs ros-noetic-industrial-msgs ros-noetic-industrial-robot-status-controller socat ros-noetic-soem python3-pymodbus ros-noetic-socketcan-interface ros-noetic-socketcan-bridge ros-noetic-joint-state-publisher-gui


#conda install -c defaults -c pytorch rospkg numpy jsonpickle scipy easydict torch==1.7.1+cu101 torchvision==0.8.2+cu101 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html
conda create -n ENV_NAME -f conda_env/environment.yml
conda activate ENV_NAME
cd ROS
rosdep init
rosdep install --from-paths src -iv
catkin build
```
## Use Unity with action server
```bash
roslaunch ur_moveit tcp_control.launch
```
> Make sure that the firewall settings are open for Unity
> Make sure that action_server node in tcp_control.launch is NOT ignored

## Use Unity as a digital twin for URSIM
```bash
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=ROBOT_IP
```
```bash
roslaunch ur_moveit tcp_control.launch
```

publish to /scaled_pos_joint_traj_controller/follow_joint_trajectory (/goal)

> Make sure that the firewall settings are open for Unity
> Make sure that action_server node in tcp_control.launch is ignored

Check if the controller are running with 
```bash
rosrun controller_manager controller_manager list
```-->