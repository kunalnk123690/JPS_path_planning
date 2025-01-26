#!/bin/bash
rm -r build/ devel/ src/CMakeLists.txt .catkin_workspace
clear
catkin_make
source devel/setup.bash
roslaunch jps_path_planning plan_path.launch
