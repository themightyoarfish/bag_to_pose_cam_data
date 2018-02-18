cmake_minimum_required(VERSION 3.1)
project(bag_to_cam_pose_data)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rosbag
    roscpp
    sensor_msgs
    geometry_msgs
    message_filters
)

catkin_package(
    CATKIN_DEPENDS
    sensor_msgs
    geometry_msgs
    INCLUDE_DIRS
    include
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
## Compiler Options
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -w")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -g -O0 -Wall -Wextra -fno-stack-protector")
## the flag for using the gold linker is only for gcc, won't work on clang
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -fuse-ld=gold -fopenmp -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/
)
include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)


add_executable(
    bag_to_cam_pose_data
    src/bag_to_cam_pose_data.cpp
)

## ADD DEPENDENCIES TO ALL TARGETS
foreach(TARGET
    bag_to_cam_pose_data
)
    add_dependencies(
        ${TARGET} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
endforeach()


## Specify libraries to link a library or executable target against

## catkin libs
# link all targets against catkin libs
foreach(TARGET
    bag_to_cam_pose_data
)
    target_link_libraries(${TARGET} ${catkin_LIBRARIES})
endforeach()
