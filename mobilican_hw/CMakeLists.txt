cmake_minimum_required(VERSION 2.8.3)
project(mobilican_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        controller_manager
        control_toolbox
        ric_interface_ros
        lpf_ros
        bms_interface
        roboteq_control
        mobilican_msgs
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
catkin_package(
          INCLUDE_DIRS include
        #  LIBRARIES lizi_hardware_interface
          CATKIN_DEPENDS
                roscpp
                ric_interface_ros
                lpf_ros
                bms_interface
                roboteq_control
                mobilican_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node

        src/hw_node.cpp
        src/mobile_robot.cpp
        src/hardware/wheel/velocities_lpf.cpp
        src/hardware/wheel/wheels_control.cpp
        src/hardware/ric_client.cpp
        src/hardware/bms.cpp
        src/robots_impl/lizi_2.cpp
        src/robots_impl/komodo_2.cpp
        src/hardware/roboteq_client.cpp

        )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )
