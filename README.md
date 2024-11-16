# DuckieTown
This repo is set for DuckieTown project QuackCruiser at ETH 2024Fall.
## How to set up the repo
In ssh out of container:  
`$ cd ~/vnc-docker/user_code_mount_dir`  
`$ git clone git@github.com:li-yunwen/DuckieTown.git project`  
In container:  
`$ cd /code/catkin_ws`    
`$ source devel/setup.bash`  
`$ cakin build`  
`$ cd src/user_code/project`  
`$ pip install -r pip_requirements.txt`  

## How to develop
**Always list the pip installed environment in pip_requirements.txt.**  

Find TODO in existing files or create new ROS packages.

