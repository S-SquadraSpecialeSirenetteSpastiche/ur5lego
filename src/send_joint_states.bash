#!/bin/bash

# 1.82471, -5.79841, 6.23306, -1.59967, 5.09219, 2.85123
# -1.00054, -3.00216, 5.02647, -0.68429, 2.0476, -3.10048

rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [-1.00054, -3.00216, 5.02647, -0.68429, 2.0476, -3.10048],layout: {dim:[], data_offset: 0"}}
