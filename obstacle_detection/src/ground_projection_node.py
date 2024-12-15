#!/usr/bin/env python3

import os
import rospy
import yaml
import numpy as np
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Segment, SegmentList
from geometry_msgs.msg import Point as PointMsg
from sensor_msgs.msg import CameraInfo
from image_processing.ground_projection_geometry import GroundProjectionGeometry, Point
from image_processing.rectification import Rectify

class SimpleGroundProjectionNode(DTROS):
    def __init__(self, node_name):
        super(SimpleGroundProjectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.bridge = CvBridge()
        self.ground_projector = None
        self.rectifier = None
        self.homography = self.load_homography()

        self.camera_info_received = False

        # Subscribers
        self.sub_camera_info = rospy.Subscriber("~camera_info", CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_segments = rospy.Subscriber("~lineseglist_in", SegmentList, self.cb_segments, queue_size=1)

        # Publishers
        self.pub_segments = rospy.Publisher("~lineseglist_out", SegmentList, queue_size=1)

    def load_homography(self):
        cali_file = os.path.join("/data/config/calibrations/camera_extrinsic", rospy.get_namespace().strip('/') + ".yaml")
        if not os.path.isfile(cali_file):
            cali_file = "/data/config/calibrations/camera_extrinsic/default.yaml"
        if not os.path.isfile(cali_file):
            rospy.logerr("Calibration file not found!")
            rospy.signal_shutdown("No calibration file.")
        with open(cali_file, 'r') as f:
            calib_data = yaml.safe_load(f)
        return np.array(calib_data['homography']).reshape(3, 3)

    def cb_camera_info(self, msg):
        if not self.camera_info_received:
            self.rectifier = Rectify(msg)
            self.ground_projector = GroundProjectionGeometry(msg.width, msg.height, self.homography)
            self.camera_info_received = True

    def pixel_to_ground(self, pixel_msg):
        norm_pt = Point.from_message(pixel_msg)
        pixel = self.ground_projector.vector2pixel(norm_pt)
        rect_pixel = self.rectifier.rectify_point(pixel)
        ground_pt = self.ground_projector.pixel2ground(Point.from_message(rect_pixel))
        return PointMsg(ground_pt.x, ground_pt.y, ground_pt.z)

    def cb_segments(self, seglist_msg):
        if not self.camera_info_received:
            rospy.logwarn("CameraInfo not received yet.")
            return

        ground_seglist = SegmentList()
        ground_seglist.header = seglist_msg.header
        
        for seg in seglist_msg.segments:
            ground_seg = Segment()
            ground_seg.points[0] = self.pixel_to_ground(seg.pixels_normalized[0])
            ground_seg.points[1] = self.pixel_to_ground(seg.pixels_normalized[1])
            ground_seg.color = seg.color
            ground_seglist.segments.append(ground_seg)

        self.pub_segments.publish(ground_seglist)

if __name__ == "__main__":
    rospy.init_node("simple_ground_projection_node", anonymous=False)
    node = SimpleGroundProjectionNode(node_name="simple_ground_projection_node")
    rospy.spin()
