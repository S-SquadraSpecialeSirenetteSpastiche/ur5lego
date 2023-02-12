cmake_minimum_required(VERSION 3.0.2)
project(ur5lego)


set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/eigen3 -w")

find_package(catkin REQUIRED COMPONENTS roscpp rospy pinocchio)

catkin_package(CATKIN_DEPENDS)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ur5lego_inverse_kinematics src/inverse_kin.cpp)
target_link_libraries(ur5lego_inverse_kinematics ${catkin_LIBRARIES})