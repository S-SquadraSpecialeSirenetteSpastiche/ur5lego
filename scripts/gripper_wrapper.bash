#!/bin/bash


# Publish the float value to the gripper_joint_position topic
rostopic pub gripper_joint_position std_msgs/Float64 "data: $1" -1
