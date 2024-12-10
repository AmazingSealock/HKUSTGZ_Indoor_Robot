cmake_minimum_required(VERSION 3.0.2)
project(gbx_manual)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        pcl_ros
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        std_srvs
        actionlib_msgs
        costmap_2d
        navigation_msgs
        ranger_msgs
)

catkin_package(
          INCLUDE_DIRS include
          LIBRARIES gbx_manual
          CATKIN_DEPENDS
            nav_msgs
            roscpp
            std_msgs
            pcl_ros
            std_srvs
            actionlib_msgs
            costmap_2d
            navigation_msgs
            ranger_msgs
          DEPENDS
)

find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS})

add_library(gbx_manual
        src/GBXManual.cpp
        src/NavigationMonitor.cpp
        )

target_link_libraries(gbx_manual ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(gbx_manual_node
        src/main.cpp
        src/GBXManual.cpp
        src/NavigationMonitor.cpp)
target_link_libraries(gbx_manual_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})