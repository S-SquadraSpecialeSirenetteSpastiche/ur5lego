#!/bin/bash

# Publish the message using rostopic
rostopic pub /ur5/joint_group_pos_controller/command std_msgs/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],layout: {dim:[], data_offset: 0"}} -1
