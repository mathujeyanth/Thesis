# Troubleshooting

Required apt packages:
- vim iputils-ping net-tools python3-pip ros-noetic-robot-state-publisher ros-noetic-moveit ros-noetic-rosbridge-suite ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf* ros-noetic-gazebo-ros-pkgs ros-noetic-joint-state-publisher dos2unix git

Required python packages:
- rospkg numpy jsonpickle scipy easydict torch

Gripper does not work with Project if set to Temporal Gauss Seidel
 - Settings > Physics > Solver Type > Projected Gauss Seidel