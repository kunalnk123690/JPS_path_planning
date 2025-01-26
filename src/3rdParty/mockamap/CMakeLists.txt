cmake_minimum_required(VERSION 2.8.3)
project(mockamap)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS})


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
