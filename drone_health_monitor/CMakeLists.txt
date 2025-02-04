cmake_minimum_required(VERSION 3.0.2)
project(drone_health_monitor)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  mavros_msgs
  message_generation
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DroneHealth.msg
  SubsystemStatus.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  diagnostic_msgs
  mavros_msgs
  std_msgs
)

catkin_package(
 LIBRARIES drone_health_monitor
 CATKIN_DEPENDS diagnostic_msgs mavros_msgs rospy std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
