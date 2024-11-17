cmake_minimum_required(VERSION 3.10)
project(navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        sensor_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
        )

add_service_files(
        FILES
        pub_trajectory.srv
)

add_message_files(
        FILES
)
#
#add_action_files(
#        FILES
#)
# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        sensor_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
)
