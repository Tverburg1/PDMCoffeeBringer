cmake_minimum_required(VERSION 3.0.2)
project(youbot_motion_controller)

## Find catkin macros and libraries and also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    trajectory_msgs
    youbot_msgs
    gazebo_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(youbot_motion_controller_node src/youbot_motion_controller_node.cpp src/youbot_motion_controller.cpp)
add_executable(youbot_configuration_publisher src/youbot_configuration_publisher.cpp src/youbot_motion_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(youbot_motion_controller_node ${catkin_LIBRARIES})
target_link_libraries(youbot_configuration_publisher ${catkin_LIBRARIES})
