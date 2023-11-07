#!/bin/bash

rostopic pub /move_channel std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 7
    stride: 1
  data_offset: 0
data: [$1, $2, $3, $4, $5, $6, $7]" -1