#!/bin/bash

# homing position
# rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [-0.32, -0.78, -2.56, -1.63, -1.57, 3.49],layout: {dim:[], data_offset: 0"}}


# no idea
# 1.82471, -5.79841, 6.23306, -1.59967, 5.09219, 2.85123
# -1.00054, -3.00216, -1.25671, -0.68429, 2.0476, -3.10048

# no idea
# rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [-1.0, -3.0, -1.25, -0.69, -4.23, -3.18],layout: {dim:[], data_offset: 0"}} -1

# debug (shoud go up and down a little)
# rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [-1.05088, -3.20991, -5.19359, 2.12031, -2.62168, 0],layout: {dim:[], data_offset: 0"}} -1
# rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [-1.05088, -2.61794, -5.96346, 2.29821, -2.62168, 0],layout: {dim:[], data_offset: 0"}} -1

# debug (close to homing position)
rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [0.656705, -4.73942, 5.89527, -4.29745, -2.2275, 3.14],layout: {dim:[], data_offset: 0"}} -1
    