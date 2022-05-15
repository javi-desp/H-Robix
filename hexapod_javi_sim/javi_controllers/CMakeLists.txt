cmake_minimum_required(VERSION 3.0.2)
project(javi_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamixel_sdk
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
    SetGroupMotorData.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetGroupMotorData.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES javi_controllers
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamixel_sdk
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/javi_controllers.cpp
# )

## Declare a C++ executable
add_executable(dinamixel_motor_controllers src/dinamixel_motor_controllers.cpp)
add_dependencies(dinamixel_motor_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dinamixel_motor_controllers ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS dinamixel_motor_controllers
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
