#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import os

class BGR2RGBNode:
    node_name: str
    robot_name: str

    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")


        # Subscriber and Publisher
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera_node/image/compressed", CompressedImage, self._image_callback)
        self.image_pub = rospy.Publisher(f"/{self.robot_name}/bgr2rgb_node/image/compressed", CompressedImage, queue_size=10)

    def _image_callback(self, image_msg: CompressedImage) -> None:
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if bgr_image is None:
                rospy.logerr(f"[{self.node_name}] Failed to decode image!")
                return

            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

            # Re-encode the image as JPEG
            _, encoded_image = cv2.imencode('.jpeg', rgb_image)
            rgb_image_msg = CompressedImage()
            rgb_image_msg.header = image_msg.header
            rgb_image_msg.format = "jpeg"
            rgb_image_msg.data = encoded_image.tobytes()

            # Publish the RGB image
            self.image_pub.publish(rgb_image_msg)
            rospy.loginfo_once(f"[{self.node_name}] Successfully processed and published RGB image.")
        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Error processing image: {e}")


if __name__ == "__main__":
    rospy.init_node("bgr2rgb_node")
    bgr2rgb_node = BGR2RGBNode()
    rospy.spin()
