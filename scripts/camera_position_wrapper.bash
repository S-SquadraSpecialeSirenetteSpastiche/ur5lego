#!/bin/bash
rostopic pub /camera_position_channel ur5lego/Pose "legoType: 0
position:
  x: 0.2
  y: 0.1
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 1.57" -1