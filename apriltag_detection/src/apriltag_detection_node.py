#!/usr/bin/env python3

# This node publishes the detected AprilTag id
# , the grid coordinates of the detected AprilTag
# and the pose of the detected AprilTag in the camera frame

import rospy
import numpy as np
from dt_apriltags import Detector
import yaml
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from apriltag_detection.msg import ApriltagMsg


class AprilTagDetectionNode:
    def __init__(self) -> None:
        self.map = self._read_map()

        self.detector = Detector(searchpath=["apriltags"],
                                    families="tag36h11",
                                    nthreads=1,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
        
        self.image_sub = rospy.Subscriber("/ivy/camera_node/image/compressed", Image, self.__image_callback)
        self.tag_pub = rospy.Publisher("/ivy/apriltag_detection_node/tag_info", ApriltagMsg, queue_size=10)

    def _read_map(self) -> np.ndarray:
        yaml_path = rospy.get_param("~map_yaml_path")
        if yaml_path is None:
            raise ValueError("map_yaml_path is not set")
        rospy.loginfo("Map YAML path: %s", yaml_path)
        
        with open(yaml_path, "r") as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            return np.array(data["map"])
        
    def __image_callback(self, image_msg: Image) -> None:
        image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        tags = self.detector.detect(image, estimate_tag_pose=True, camera_params=None, tag_size=None)
        
        for tag in tags:
            tag_id = tag.tag_id
            # TODO: Get the grid coordinates of the detected AprilTag, which can be stored in anther YAML file
            # TODO: Get the pose of the detected AprilTag in the camera frame which can be used to determine which tag to pick? Idk but we definitely
            # need a way to deal with multiple tags in the camera frame
                    
            tag_msg = ApriltagMsg()
            tag_msg.tag_id = tag_id
            # TODO: Publish the tag_id, grid coordinates and pose of the detected AprilTag
            # ApriltagMsg:
            # geometry_msgs/PoseStamped tag_pose
            # int32 tag_id
            # int32[2] grid_coords
            self.tag_pub.publish(tag_msg)
