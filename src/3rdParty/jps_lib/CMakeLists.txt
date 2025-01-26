cmake_minimum_required(VERSION 3.0.2)
project(jps_lib)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jps_lib
  CATKIN_DEPENDS roscpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


file(GLOB_RECURSE LIB_FILES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${LIB_FILES})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${catkin_LIBRARIES})


install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)