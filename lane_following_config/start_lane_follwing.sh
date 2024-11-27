#!/bin/bash
# run in main-workspace container
rostopic pub --once /${VEHICLE_NAME}/joy_mapper_node/joystick_override duckietown_msgs/BoolStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
data: false"