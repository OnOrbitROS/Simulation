cmake_minimum_required(VERSION 3.0.2)
project(orbit_publisher_pkg)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  orbit_msgs
)

find_package (Eigen3 3.3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orbit_publisher_pkg
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs orbit_msgs
  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/orbit_publisher_pkg_node.cpp src/Orbit.cpp)
target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 Eigen3::Eigen
)

add_executable(fix_${PROJECT_NAME}_node src/fix_orbit_publisher_pkg_node.cpp src/Orbit.cpp)
target_link_libraries(fix_${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 Eigen3::Eigen
)
