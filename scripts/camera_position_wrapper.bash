#!/bin/bash
rostopic pub /camera_position_channel ur5lego/Pose "legoType: 0
position:
  x: -0.45
  y: 0.35
  z: 0.60
orientation:
  x: 0.0
  y: -1.57
  z: 1.57" -1