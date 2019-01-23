cmake_minimum_required(VERSION 2.8.3)
project(pick_and_place)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        actionlib
        std_msgs
        sensor_msgs
        geometry_msgs
        image_transport
        cv_bridge
        pcl_conversions
        pcl_ros
        tf
        dynamic_reconfigure
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        kdl_parser
        actionlib
        tf_conversions
        trajectory_msgs
        gazebo_ros
        ar_track_alvar
        mobilican_msgs
        gazebo_dev
        pluginlib
        message_generation
        genmsg
)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED )

generate_dynamic_reconfigure_options(
        cfg/FindObjectDynParam.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        roscpp
        actionlib
        std_msgs
        sensor_msgs
        geometry_msgs
        dynamic_reconfigure
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        gazebo_ros
        message_runtime
        std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


add_library(gazebo_grasp_fix SHARED
        src/GazeboGraspFix.cpp
        src/GazeboGraspGripper.cpp
        include/pick_and_place/GazeboGraspFix.h
        include/pick_and_place/GazeboGraspGripper.h)
add_dependencies(gazebo_grasp_fix ${catkin_EXPORTED_TARGETS})

add_executable(point_head_action src/point_head_action.cpp)
add_dependencies(point_head_action mobilican_msgs_generate_messages_cpp)
add_dependencies(point_head_action ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(point_head_action mobilican_msgs_generate_messages_cpp)
target_link_libraries(point_head_action ${catkin_LIBRARIES} )

add_executable(find_objects_node src/find_objects.cpp)
add_dependencies(find_objects_node mobilican_msgs_generate_messages_cpp)
add_dependencies(find_objects_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(find_objects_node ${PROJECT_NAME}_gencfg)
target_link_libraries(find_objects_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(pan_tilt_object_tracking_node src/pan_tilt_object_tracking.cpp)
add_dependencies(pan_tilt_object_tracking_node mobilican_msgs_generate_messages_cpp)
add_dependencies(pan_tilt_object_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pan_tilt_object_tracking_node ${catkin_LIBRARIES})

add_executable(drive2object_node src/drive2object.cpp)
add_dependencies(drive2object_node mobilican_msgs_generate_messages_cpp)
add_dependencies(drive2object_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(drive2object_node ${catkin_LIBRARIES})

add_executable(drive_and_pick src/drive_and_pick.cpp)
add_dependencies(drive_and_pick mobilican_msgs_generate_messages_cpp)
add_dependencies(drive_and_pick ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(drive_and_pick ${catkin_LIBRARIES})


add_executable(pick_and_place_node
        src/pick_and_place_node.cpp)
add_dependencies(pick_and_place_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(
        pick_and_place_node
        ${catkin_LIBRARIES}
)

add_executable(objects_handler src/objects_handler.cpp)
add_dependencies(objects_handler ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(objects_handler ${catkin_LIBRARIES})

target_link_libraries(gazebo_grasp_fix
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES}      )


install(TARGETS gazebo_grasp_fix
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        )