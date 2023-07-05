#!/bin/bash

sleep 3
rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray \
    "{data: [-0.32, -0.78, -2.56, -1.63, -1.57, 3.49],layout: {dim:[], data_offset: 0"}} -1
