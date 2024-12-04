#!/usr/bin/env python3

# This node controls the vehicle to follow the lane or stop
import rospy
import os
from apriltag_detection.msg import ApriltagMsg
from enum import Enum
from duckietown_msgs.msg import BoolStamped, WheelsCmdStamped

class State(Enum):
    STOP = 0
    FOLLOW_LANE = 1


class LaneFollowingControllerNode:
    node_name: str
    robot_name: str

    state: State

    planner_cmd_sub: rospy.Subscriber
    apriltag_sub: rospy.Subscriber

    lane_follower_cmd_pub: rospy.Publisher
    wheel_cmd_pub: rospy.Publisher


    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        self.state = State.STOP

        self.planner_cmd_sub = rospy.Subscriber(f"/{self.robot_name}/planner_node/lane_following_cmd", BoolStamped, self._cmd_callback)
        self.apriltag_sub = rospy.Subscriber(f"/{self.robot_name}/apriltags_detector_node/tag_info", ApriltagMsg, self._apriltag_callback)

        self.lane_follower_cmd_pub = rospy.Publisher(f"/{self.robot_name}/joy_mapper_node/joystick_override", BoolStamped, queue_size=1)
        self.wheel_cmd_pub = rospy.Publisher(f"/{self.robot_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
    
    def _cmd_callback(self, msg: BoolStamped) -> None:
        if msg.data:
            if self.state == State.STOP:
                self.state = State.FOLLOW_LANE
                rospy.loginfo(f"[{self.node_name}] Lane following is enabled.")

                send_msg = BoolStamped()
                send_msg.header.stamp = rospy.Time.now()
                send_msg.header.frame_id = ''
                send_msg.data = True
                self.lane_follower_cmd_pub.publish(send_msg)
        else:
            if self.state == State.FOLLOW_LANE:
                self.state = State.STOP
                rospy.loginfo(f"[{self.node_name}] Lane following is disabled.")

                send_msg = BoolStamped()
                send_msg.header.stamp = rospy.Time.now()
                send_msg.header.frame_id = ''
                send_msg.data = False
                self.lane_follower_cmd_pub.publish(send_msg)

    def _apriltag_callback(self, msg: ApriltagMsg) -> None:
        if self.state == State.FOLLOW_LANE:
            return
        if msg.tag_pose.pose.position.z <= 0.3:
            # TODO: let a PID controller to control the vehicle to stop at 30cm to the tag
            # For now just emergency stop the vehicle
            send_msg = WheelsCmdStamped()
            send_msg.header.stamp = rospy.Time.now()
            send_msg.header.frame_id = ''
            send_msg.vel_left = 0.0
            send_msg.vel_right = 0.0
            self.wheel_cmd_pub.publish(send_msg)

if __name__ == "__main__":
    rospy.init_node("lane_following_controller_node")
    lane_following_controller_node = LaneFollowingControllerNode()
    rospy.spin()