cmake_minimum_required(VERSION 2.8.3)
project(jetyak_uav_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
	std_srvs
  tf
  tf2
  dji_sdk
  visualization_msgs
  message_generation
  actionlib_msgs
)

find_package(DJIOSDK REQUIRED)

## Custom message files to be created
add_message_files(DIRECTORY msg
  FILES
  Waypoint.msg
	WaypointArray.msg
)
add_service_files(DIRECTORY srv
  FILES
  FourAxes.srv
	Int.srv
  GetString.srv
  SetString.srv
  TakeoffParams.srv
  LandParams.srv
  ReturnParams.srv
	SetWaypoints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/FollowConstants.cfg
#  cfg/LandConstants.cfg
#)
## includes, libraries, depends
catkin_package(
  INCLUDE_DIRS include lib/bsc_common/include
  LIBRARIES jetyak_uav_utils
  CATKIN_DEPENDS ar_track_alvar geometry_msgs ar_track_alvar_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2 tf2_geometry_msgs dji_sdk visualization_msgs message_runtime
  DEPENDS system_lib
)

## Includes
include_directories(
  include
  lib/bsc_common/include
  ${catkin_INCLUDE_DIRS}
)

## Add executables
add_executable(dji_pilot_node
  src/dji_pilot.cpp
)

add_executable(gimbal_tag_node
  src/gimbal_tag.cpp
)

add_executable(behaviors_node
  src/behaviors_services.cpp
  src/behaviors_behaviors.cpp
  src/behaviors_callbacks.cpp
	src/behaviors_common.cpp
  src/behaviors_main.cpp
  lib/bsc_common/pid.cpp
  lib/bsc_common/util.cpp
)

#add dependencies
add_dependencies(dji_pilot_node ${catkin_EXPORTED_TARGETS} )
add_dependencies(gimbal_tag_node ${catkin_EXPORTED_TARGETS} )
add_dependencies(behaviors_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp) #${PROJECT_NAME}_gencfg )

## Link executables
target_link_libraries(dji_pilot_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

target_link_libraries(gimbal_tag_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

target_link_libraries(behaviors_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)
