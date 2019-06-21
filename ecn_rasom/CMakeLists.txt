cmake_minimum_required(VERSION 2.8.3)
project(ecn_rasom)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
  visp_bridge
  message_generation
)

add_message_files(
  FILES
  beacons_ping.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(waypoint src/waypoint.cpp)
add_executable(beacons src/beacons.cpp)
add_executable(beacons_trilateration src/beacons_trilateration.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(waypoint ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(beacons ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(beacons_trilateration ${catkin_LIBRARIES} yaml-cpp)
