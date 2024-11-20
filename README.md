# DuckieTown
This repo is set for DuckieTown project QuackCruiser at ETH 2024Fall.
## How to set up the repo
In ssh out of container:  
`$ cd ~/vnc-docker/user_code_mount_dir`  
`$ git clone git@github.com:li-yunwen/DuckieTown.git project`  
To update the latest repo:  
`$ git pull` 

In container:  
`$ cd /code/catkin_ws`    
`$ source devel/setup.bash`  
`$ cakin build`  
`$ cd src/user_code/project`  
`$ pip install -r pip_requirements.txt`  
In `project_bringup.sh` , change `robot_name="ivy"` to your robot's name.  


## How to develop
**Always list the pip installed environment in pip_requirements.txt.**  
Keep everthing in SI unit.  

Always read robot_name from rosparam instead of hardcode it in the node script:
```
self.robot_name = rospy.get_param("/robot_name")
if self.robot_name is None:
    raise ValueError("/robot_name is not set, run project_bringup.sh first.")
rospy.loginfo("Robot name: %s", self.robot_name)
```

Find TODO in existing files or create new ROS packages.  

## How to run  
Everytime after rebooting the robot:
`$ cd /code/catkin_ws/src/user_code/project`  
`$ ./project_bringup.sh`  
This will create a ros parameter for your robot's name, which will be used in the project. 

Don't forget to source `devel/setup.bash` in RealVNC terminals if you cannot locate a package.

Don't forget to `chmod +x` for shell scripts and python files.  

#### AprilTag Detection  
1. Perform camera calibration according to [the tutorial](https://github.com/ETHZ-DT-Class/camera-calibration-tools?tab=readme-ov-file).

2. In `/code/catkin_ws/src/user_code/project/apriltag_detection/launch/apriltag_detection.launch`, change `ivy.yaml` to your robot's intrinsic parameter file.

3. launch:  
`roslaunch apriltag_detection apriltag_detection.launch`

4. You should then be able to echo the topic `/[ROBOT_NAME]/apriltag_detection_node/tag_info`. You can visualize `/[ROBOT_NAME]/apriltag_detection_node/tag_info/overlay/compressed`:  
![detecetd tags](README_asset/detected_tags.png)