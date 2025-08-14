colcon build --packages-select diablo_pathtracking --cmake-clean-cache

ros2 run diablo_pathtracking diablo_track_node --ros-args -p strategy:=pd
ros2 run diablo_pathtracking test_track_node --ros-args -p path_type:=line
