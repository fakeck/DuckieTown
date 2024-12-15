#! /usr/bin/env python3

import rospy
from enum import Enum
import os
from darknet_ros_msgs.msg import BoundingBoxes
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped



#print("obstacle detection initialized")


class ObstacleDetectionNode:
    node_name: str
    robot_name: str

    obstacle_detection_pub: rospy.Publisher

    def __init__(self) -> None:

        #init ros node
        self.node_name = rospy.get_name()
        self.obstacle_detected = False
        self.conversion_rate = 10 #Hz
        self.noMessageTime = rospy.get_time()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        #generalize robot name used
        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        #Subscribers
        self.boundingbox_sub = rospy.Subscriber(f"/darknet_ros/bounding_boxes", BoundingBoxes, self._obstacle_detection_callback)

        #Publisher for detected obstacles (only active when detected)
        self.obstacle_detected_pub = rospy.Publisher(f"/{self.robot_name}/obstacle_detection_node/obstacle_detected", BoolStamped, queue_size=1)
        
        #Timer to publish every 0.5sec, so we know if no obstacle is detected
        self.timer = rospy.Timer(rospy.Duration(1/self.conversion_rate), self._obstacle_detection_publish)



    #if obstacle detected publish 
    def _obstacle_detection_callback(self, msg: BoundingBoxes) -> None:
        
        self.noMessageTime = rospy.get_time()

        temp_obstacle_detected = False

        for i in range(len(msg.bounding_boxes)):

            #TODO: ignore persons and chairs
            
            #print(f"THIS IS YMAX {msg.bounding_boxes[i].ymax}")

            #print(f"CLASS:  {msg.bounding_boxes[i].Class}")

            #Y-axis points downwards in image plane -> ymax is closest point of bbox to duckie
            if msg.bounding_boxes[i].ymax >= 200 and msg.bounding_boxes[i].xmax <= 520 and msg.bounding_boxes[i].xmin >= 120: 
                #print("Obstacle detected! Stopping the Duckie.")
                temp_obstacle_detected = True
                break
            else:
                #print("obstacle out of Range - no Danger")
                pass

        self.obstacle_detected = temp_obstacle_detected
        
        #publish every time callback is called
        self.obstacle_detected_pub.publish(BoolStamped(header=rospy.Header(), data = self.obstacle_detected))




    #necessary because _obstacle_detection_callback is only called when yolo detects obstacle -> we need to know when there is no obstacle
    def _obstacle_detection_publish(self, event) -> None:

        #when there is 0.5 seconds no publish from yolo, we know there is no obstacle
        if rospy.get_time() - self.noMessageTime > 0.5:
            self.obstacle_detected = False

        #publish
        self.obstacle_detected_pub.publish(BoolStamped(header=rospy.Header(), data = self.obstacle_detected))
        





if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node")
    obstacle_detection_node = ObstacleDetectionNode()
    rospy.spin()