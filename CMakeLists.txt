cmake_minimum_required(VERSION 3.0.2)
project(inverse_kin)


set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/eigen3 -w")

find_package(catkin REQUIRED COMPONENTS roscpp rospy pinocchio)

catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

