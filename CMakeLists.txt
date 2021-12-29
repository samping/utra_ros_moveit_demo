cmake_minimum_required(VERSION 2.8.3)
project(utra_ros_moveit_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  trajectory_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES probot_demo
   CATKIN_DEPENDS roscpp rospy moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(moveit_fk_demo src/moveit_fk_demo.cpp)
target_link_libraries(moveit_fk_demo ${catkin_LIBRARIES})

add_executable(moveit_ik_demo src/moveit_ik_demo.cpp)
target_link_libraries(moveit_ik_demo ${catkin_LIBRARIES})




## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(PROGRAMS 
   scripts/moveit_fk_demo.py
   scripts/moveit_ik_demo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
   moveit_fk_demo 
   moveit_ik_demo 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


