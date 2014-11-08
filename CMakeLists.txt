set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

cmake_minimum_required(VERSION 2.8.3)
project(s8_wall_follower_controller)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs s8_msgs s8_common_node s8_motor_controller s8_ir_sensors actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES FollowWall.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp s8_msgs geometry_msgs s8_common_node s8_motor_controller s8_ir_sensors actionlib_msgs actionlib
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(s8_wall_follower_controller_node src/wall_follower_controller_node.cpp)
target_link_libraries(s8_wall_follower_controller_node ${catkin_LIBRARIES})
add_dependencies(s8_wall_follower_controller_node ${s8_motor_controller_EXPORTED_TARGETS})
