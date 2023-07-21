cmake_minimum_required(VERSION 3.0.2)
project(RoboticsProject_2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  map_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  roslib
)
add_service_files(
		FILES
		save_map.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)




####


###################################
## catkin specific configuration ##
###################################

catkin_package(

CATKIN_DEPENDS 
  roscpp
  std_msgs 
  message_runtime
  roslib
)
#######
## Build ##
###########


include_directories(
 include ${catkin_INCLUDE_DIRS}
)


add_executable(OdometryBroadcaster src/OdometryBroadcaster.cpp)
target_link_libraries(OdometryBroadcaster ${catkin_LIBRARIES})
add_dependencies(OdometryBroadcaster ${catkin_EXPORTED_TARGETS})

add_executable(PathBroadcaster src/PathBroadcaster.cpp)
target_link_libraries(PathBroadcaster ${catkin_LIBRARIES})
add_dependencies(PathBroadcaster ${catkin_EXPORTED_TARGETS})

add_executable(CustomMapSaver src/CustomMapSaver.cpp)
target_link_libraries(CustomMapSaver ${catkin_LIBRARIES})
add_dependencies(CustomMapSaver ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/occupancy_grid_processing.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)