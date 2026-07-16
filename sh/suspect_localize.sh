#!/bin/bash
xterm -hold -e "source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash;  
ros2 run suspect_matcher suspect_localizer --ros-args \
  -p location_source:=amcl_pose \
  -p min_valid_points:=20 \
  -p tf_timeout_sec:=1.0
 ; exec bash"
# location_source=amcl_pose (default): suspect is placed at the robot's own
#   /amcl_pose at the capture moment — cheap, no depth/tf. Drive up to the person.
# For depth-based placement instead, use:
#   -p location_source:=pointcloud -p cloud_wait_sec:=2.0 -p min_valid_points:=20
