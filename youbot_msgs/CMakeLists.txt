cmake_minimum_required(VERSION 3.0.2)
project(youbot_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    rospy
)

add_message_files(
    FILES
    Control.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)
