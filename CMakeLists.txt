cmake_minimum_required(VERSION 3.0.2)
project(ur5lego)

set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/eigen3 -w")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    roslib 
    pinocchio 
    actionlib_msgs 
    actionlib
    gazebo_ros
    std_msgs
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}") #questo rompe tutto ancora di più
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_action_files(
    DIRECTORY action
    FILES Move.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs roslib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${catkin_LIBRARY_DIRS}
    ${GAZEBO_LIBRARY_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)


add_library(math src/math_tools.cpp)
add_library(inverse_kinematics src/inverse_kinematics.cpp)
add_library(trajectory src/trajectory_planner.cpp)
add_library(link_controller_plugin SHARED  src/link_controller_plugin.cpp)

target_link_libraries(trajectory ${catkin_LIBRARIES} math inverse_kinematics)
target_link_libraries(link_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_executable(ur5lego_move_server src/move_server.cpp)
add_executable(ur5lego_move_client src/move_client.cpp)
add_executable(ur5lego_send_homing_position src/send_homing_position.cpp)
add_executable(ur5lego_move_server_wrapper src/move_server_wrapper.cpp)

target_link_libraries(ur5lego_move_server ${catkin_LIBRARIES} pinocchio::pinocchio inverse_kinematics math trajectory)
target_link_libraries(ur5lego_move_client ${catkin_LIBRARIES})
target_link_libraries(ur5lego_send_homing_position ${catkin_LIBRARIES} trajectory)
target_link_libraries(ur5lego_move_server_wrapper ${catkin_LIBRARIES})
