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
    ros_impedance_controller
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_message_files(
    FILES
    ConnectLinks.msg
)

add_action_files(
    DIRECTORY action
    FILES Move.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs
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


add_library(math src/movement/math_tools.cpp)
add_library(inverse_kinematics src/movement/inverse_kinematics.cpp)
add_library(trajectory src/movement/trajectory_planner.cpp)
add_library(link_controller_plugin SHARED  src/link_controller_plugin.cpp)
add_library(cache_handler src/movement/cache_handler.cpp)

target_link_libraries(trajectory ${catkin_LIBRARIES} math inverse_kinematics)
target_link_libraries(link_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES} math cache_handler)


add_executable(ur5lego_move_server src/movement/move_server.cpp)
add_executable(ur5lego_send_homing_position src/movement/send_homing_position.cpp)
add_executable(ur5lego_move_server_wrapper src/movement/move_server_wrapper.cpp)
add_executable(ur5lego_position_publisher src/movement/position_publisher.cpp)
# add_executable(move_client_example src/movement/move_client_example.cpp)

target_link_libraries(ur5lego_move_server ${catkin_LIBRARIES} pinocchio::pinocchio inverse_kinematics math trajectory cache_handler)
target_link_libraries(ur5lego_send_homing_position ${catkin_LIBRARIES} trajectory)
target_link_libraries(ur5lego_move_server_wrapper ${catkin_LIBRARIES})
target_link_libraries(ur5lego_position_publisher ${catkin_LIBRARIES})
# target_link_libraries(move_client_example ${catkin_LIBRARIES})
