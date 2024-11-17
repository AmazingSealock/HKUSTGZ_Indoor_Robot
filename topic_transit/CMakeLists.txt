cmake_minimum_required(VERSION 3.0.2)
project(topic_transit)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        pcl_ros
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pcd2pgm
        #  CATKIN_DEPENDS nav_msgs pcl roscpp rospy std_msgs
        #  DEPENDS system_lib
)

find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

add_executable(cmd_transit src/CmdTransit.cpp)
target_link_libraries(cmd_transit ${catkin_LIBRARIES})

add_executable(cloud_transit src/CloudTransit.cpp)
target_link_libraries(cloud_transit ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(topic_gen_trajectory src/TopicGenTrajectory.cpp)
target_link_libraries(topic_gen_trajectory ${catkin_LIBRARIES} )

add_executable(point_transit src/PointTransit.cpp)
target_link_libraries(point_transit ${catkin_LIBRARIES})
