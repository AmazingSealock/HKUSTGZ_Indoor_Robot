cmake_minimum_required(VERSION 3.0.2)
project(navigation_tools)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        pcl_ros
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
        dynamic_reconfigure
        nodelet
        pluginlib
)

find_package(PCL 1.10 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/CloudFilter.cfg
)

catkin_package(
          INCLUDE_DIRS
          LIBRARIES
          CATKIN_DEPENDS
            nav_msgs
            pcl_ros
            roscpp
            std_msgs
            geometry_msgs
            sensor_msgs
            tf2
            tf2_ros
            tf2_sensor_msgs
            dynamic_reconfigure
            nodelet
            pluginlib
          DEPENDS
)

find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

add_executable(cmd_transit src/CmdTransit.cpp)
target_link_libraries(cmd_transit ${catkin_LIBRARIES})

add_executable(topic_gen_trajectory src/TopicGenTrajectory.cpp)
target_link_libraries(topic_gen_trajectory ${catkin_LIBRARIES} )

add_executable(point_transit src/PointTransit.cpp)
target_link_libraries(point_transit ${catkin_LIBRARIES})


add_library(cloud_filter_nodelet
        src/CloudIntensityFilterNodelet.cpp
        src/CloudTransitNodelet.cpp
)

target_link_libraries(cloud_filter_nodelet
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

install(FILES cloud_filter_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)