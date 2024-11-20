#!/bin/bash
robot_name="ivy"  # Change "ivy" to each robot's unique name

# Set the robot name on the ROS parameter server
rosparam set /robot_name "$robot_name"