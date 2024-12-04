#!/bin/bash
# https://colab.research.google.com/drive/1Pjb4swpO4VzAAnL67wWuVrcBkzv0F7Mn#scrollTo=lmxLaXlqCgjV
# run in ssh outside container
# docker pull duckietown/dt-core:daffy-arm64v8
# -v apriltag_config:/code/catkin_ws/src/dt-core/packages/apriltag/config/apriltag_detector_node/duckiebot.yaml
docker run \
 -it --rm -e VEHICLE_NAME=$(echo $VEHICLE_NAME) --name=dt-core --privileged --memory "800m" --memory-swap="2800m" \
 -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
 -v /home/duckie/vnc-docker/user_code_mount_dir/project/lane_following_controller/config/default.yaml:/code/catkin_ws/src/dt-core/packages/line_detector/config/line_detector_node/default.yaml \
 duckietown/dt-core:daffy-arm64v8 \
 roslaunch duckietown_demos lane_following.launch
