cmake_minimum_required(VERSION 2.8.3)
project(winch_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  message_generation
  actionlib_msgs
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    Depth.msg
)

# add_service_files(
#   FILES
# )

#add_action_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    message_runtime
    actionlib_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)