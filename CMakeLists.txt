cmake_minimum_required(VERSION 2.8.3)
project(geomagic_touch_x_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/OpenHaptics/Developer/3.4-0/libsrc/include/
  src/
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  /opt/OpenHaptics/Developer/3.4-0/libsrc/include/
  src/
)

add_executable(geomagic_touch_x_node src/geomagic_touch_x_node.cpp)
target_link_libraries(
  geomagic_touch_x_node
  ${catkin_LIBRARIES}
  HDU
  HD
)
