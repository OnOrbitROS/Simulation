cmake_minimum_required(VERSION 3.0.2)
project(orbit_robot_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package (Eigen3 3.3 REQUIRED)
find_package(gazebo REQUIRED)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
)

add_library(Orbit_robot_pkg_plugin src/Orbit_robot_pkg_plugin.cpp src/OrbitLink.cpp )
target_link_libraries(Orbit_robot_pkg_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})