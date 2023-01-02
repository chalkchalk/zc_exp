cmake_minimum_required(VERSION 3.10.1)
project(zc_exp)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        tf
        )

find_package(Eigen3)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rc_sim_oub
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

add_library(exp_lib
        src/experiment_node.cpp
        src/experiment_node.h
        src/follower.cpp
        src/follower.h
        src/utils.cpp
        src/utils.h
        src/estimation.cpp
        src/estimation.h
        src/leader.cpp
        src/leader.h
        src/eigenmvn.h
        )

target_link_libraries(exp_lib ${catkin_LIBRARIES})

add_executable(experiment_node src/experiment_node.cpp)
target_link_libraries(experiment_node
        ${catkin_LIBRARIES}
        exp_lib
        )
