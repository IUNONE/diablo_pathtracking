```
mkdir -p $HOME/diablo_ws/src
cd $HOME/diablo_ws/src
git clone -b basic https://github.com/DDTRobot/diablo_ros2.git
git clone -b foxy https://github.com/IUNONE/diablo_pathtracking.git
cd ..
colcon build --packages-up-to diablo_ctrl

colcon build --packages-select diablo_pathtracking

# add to ~/.bashrc
source $HOME/zsh_ws/install/setup.bash

source $HOME/diablo_ws/install/setup.bash

ros2 run diablo_ctrl diablo_ctrl_node

# open_loop / pd / pid
ros2 run diablo_pathtracking control_node --ros-args -p strategy:="pd"

# 测试用例
ros2 run diablo_pathtracking test_node --ros-args -p path_type:="line" -p path_length:=0.32 -p publish_rate:=2.0 -p point_spacing:=0.016
```