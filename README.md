# DuckieTown
This repo is set for DuckieTown project QuackCruiser at ETH 2024Fall.
## How to set up the repo
In ssh out of container:  
`$ cd ~/vnc-docker/user_code_mount_dir`  
`$ git clone git@github.com:li-yunwen/DuckieTown.git project`  
`$ nano ~/.bashrc`, add `export VEHICLE_NAME="[YOUR_ROBOT_NAME]"`.  
This will export your robot's name [YOUR_ROBOT_NAME] to the environment in new terminals.  
To update the latest repo:  
`$ git pull` 

In container:  
`$ cd /code/catkin_ws`    
`$ source devel/setup.bash`  
`$ catkin build`  
`$ cd src/user_code/project`  
`$ pip install -r pip_requirements.txt`  

## How to develop
**Always list the pip installed environment in pip_requirements.txt.**  
Keep everthing in SI unit.  

Always read robot_name from env instead of hardcode it in the node script:
```
self.robot_name = os.getenv('VEHICLE_NAME', None)
if self.robot_name is None:
    raise ValueError("$VEHICLE_NAME is not set, export it first.")
rospy.loginfo("Robot name: %s", self.robot_name)
```

Find TODO in existing files or create new ROS packages.  

## How to run  
Don't forget to source `devel/setup.bash` in RealVNC terminals if you cannot locate a package.

Don't forget to `chmod +x` for shell scripts and python files.  

#### AprilTag Detection  
1. Perform camera calibration according to [the tutorial](https://github.com/ETHZ-DT-Class/camera-calibration-tools?tab=readme-ov-file).

2. In `/code/catkin_ws/src/user_code/project/apriltag_detection/launch/apriltag_detection.launch`, change `ivy.yaml` to your robot's intrinsic parameter file.

3. launch:  
`roslaunch apriltag_detection apriltag_detection.launch`

4. You should then be able to echo the topic `/[ROBOT_NAME]/apriltag_detection_node/tag_info`. You can visualize `/[ROBOT_NAME]/apriltag_detection_node/tag_info/overlay/compressed` using `$ rqt_image_view` in RealVNC:  
![detecetd tags](README_asset/detected_tags.png)

#### Lane Following
To start:  
1. In ssh out of container:  
`$ cd /home/duckie/vnc-docker/user_code_mount_dir/project/lane_following_config`  
`$ ./bringup_dt_core.sh`  

2. In container:  
`$ cd /code/catkin_ws/src/user_code/project/lane_following_config`  
`$ ./start_lane_follwing.sh`

To stop:  
1. In container:  
`$ cd /code/catkin_ws/src/user_code/project/lane_following_config`  
`$ ./stop_lane_follwing.sh`

2. In ssh out of container:  
`$ cd /home/duckie/vnc-docker/user_code_mount_dir/project/lane_following_config`  
`$ ./shutdown_dt_core.sh`  

