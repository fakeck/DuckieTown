#! /usr/bin/env python3

import rospy
from enum import Enum
import os
from darknet_ros_msgs.msg import BoundingBoxes
from duckietown_msgs.msg import WheelsCmdStamped


print("obstacle avoidance initialized")

#TODO will be defined by state node
class State(Enum):
    DORMANT = 0
    ACTIVE = 1


class ObstacleAvoidanceNode:
    node_name: str
    robot_name: str

    state: State.ACTIVE

    state_sub: rospy.Subscriber
    obstacle_detection_sub: rospy.Subscriber

    obstacle_avoidance_cmd_pub: rospy.Publisher
    wheel_cmd_pub: rospy.Publisher


    def __init__(self) -> None:

        #init ros node
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        #generalize robot name used
        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        #only temporary
        self.state = State.ACTIVE

        # self.state_sub = rospy.Subscriber(f"/{self.robot_name}/ # TODO: wait for state message
        self.boundingbox_sub = rospy.Subscriber(f"/darknet_ros/bounding_boxes", BoundingBoxes, self._obstacle_detection_callback)

        #self.obstacle_cleared_pub = rospy.Publisher(f"/{self.robot_name}/state_machine_node/obstacle_cleared", ObstacleClearedStamped, queue_size=1)
        self.wheel_cmd_pub = rospy.Publisher(f"/{self.robot_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
    
        pass


    def _state_callback(self, state_msg) -> None:
        print("State Callback???")
        # self.state = state_msg.data
        # TODO
        pass


    def _obstacle_detection_callback(self, msg: BoundingBoxes) -> None:

        if self.state != State.ACTIVE:
            print("not ACTIVE")
            return
        
        for i in range(len(msg.bounding_boxes)):
            
            print(f"THIS IS YMAX {msg.bounding_boxes[i].ymax}")

            print(f"CLASS:  {msg.bounding_boxes[i].Class}")

            #Y-axis points downwards in image plane -> ymax is closest point of bbox to duckie
            if msg.bounding_boxes[i].ymax >= 150: 
                stop_msg = WheelsCmdStamped()
                stop_msg.vel_left = 0.0
                stop_msg.vel_right = 0.0
                self.wheel_cmd_pub.publish(stop_msg)
                print("Obstacle detected! Stopping the Duckie.")
            pass

            print("obstacle out of Range - no Danger")
            pass

if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance_node")
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rospy.spin()