#!/bin/bash
# rm -r build/ devel/ src/CMakeLists.txt .catkin_workspace
clear
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source devel/setup.bash
roslaunch jps_path_planning plan_path.launch
