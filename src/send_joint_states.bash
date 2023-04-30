#!/bin/bash

# 1.82471, -5.79841, 6.23306, -1.59967, 5.09219, 2.85123

rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray "{data: [1.82471, -5.79841, -0.0531, -1.59967, 5.09219, 2.85123],layout: {dim:[], data_offset: 0"}}
