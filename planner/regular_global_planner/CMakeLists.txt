cmake_minimum_required(VERSION 2.8.3)
project(regular_global_planner)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        base_local_planner
        costmap_2d
        geometry_msgs
        move_base
        nav_core
        nav_msgs
        pluginlib
        roscpp
        tf
        visualization_msgs
        std_srvs
        actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        base_local_planner
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pluginlib
        roscpp
        tf
        visualization_msgs
        std_srvs
        actionlib_msgs
)

add_library(regular_global_planner src/RegularGlobalPlanner.cpp)
add_dependencies(regular_global_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(regular_global_planner ${catkin_LIBRARIES})

install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
        TARGETS regular_global_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
        FILES regular_global_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
