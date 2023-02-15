cmake_minimum_required(VERSION 3.0.2)
project(ur5lego)


set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/eigen3 -w")

find_package(catkin REQUIRED COMPONENTS roscpp rospy pinocchio actionlib_msgs actionlib)

add_action_files(
    DIRECTORY action
    FILES Move.action
)

generate_messages(
    DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(ur5lego_move_server src/move_server.cpp)
target_link_libraries(ur5lego_move_server ${catkin_LIBRARIES} pinocchio::pinocchio)
# add_dependencies(ur5lego_move_server ${ur5lego_EXPORTED_TARGETS})

add_executable(ur5lego_move_client src/move_client.cpp)
target_link_libraries(ur5lego_move_client ${catkin_LIBRARIES})
# add_dependencies(ur5lego_move_client ${ur5lego_EXPORTED_TARGETS})
