#!/bin/bash
# https://colab.research.google.com/drive/1Pjb4swpO4VzAAnL67wWuVrcBkzv0F7Mn#scrollTo=lmxLaXlqCgjV
# run in ssh outside container
docker pull duckietown/dt-core:daffy-arm64v8
# this cmd is from myself
# docker run -it --rm --network=host --privileged \
#   -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
#   -v /data:/data \
#   -v /tmp:/tmp \
#   --name dt-core \
#   duckietown/dt-core:daffy-arm64v8

# given by TA
docker run \
 -it --rm --name=lane-following --net host --privileged --memory "800m" --memory-swap="2800m" \
 -v /data:/data -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
 -v /home/duckie/vnc-docker/user_code_mount_dir/project/lane_following_config/default.yaml:/code/catkin_ws/src/dt-core/packages/line_detector/config/line_detector_node/default.yaml \
 duckietown/dt-core:daffy-arm64v8 \
 roslaunch duckietown_demos lane_following.launch veh:=${VEHICLE_NAME}