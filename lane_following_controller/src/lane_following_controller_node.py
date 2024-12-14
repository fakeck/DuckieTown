#!/usr/bin/env python3

# This node controls the vehicle to follow the lane or stop
import rospy
import os
from apriltag_detection.msg import ApriltagMsg
from enum import Enum
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, WheelEncoderStamped
from std_msgs.msg import Int32
from state_machine.state_machine_node import State

class LaneFollowingControllerNode:
    node_name: str
    robot_name: str

    is_following: bool
    state_sub: rospy.Subscriber

    left_ticks: int
    right_ticks: int
    left_wheel_encoder_sub: rospy.Subscriber
    right_wheel_encoder_sub: rospy.Subscriber

    self_state_pub: rospy.Publisher
    lane_follower_cmd_pub: rospy.Publisher
    wheel_twist_pub: rospy.Publisher


    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        self.is_following = False
        self.left_ticks = 0
        self.right_ticks = 0

        self.lane_follower_cmd_pub = rospy.Publisher(f"/{self.robot_name}/joy_mapper_node/joystick_override", BoolStamped, queue_size=1)
        self.wheel_twist_pub = rospy.Publisher(f"/{self.robot_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        # indicate whether this node is brought up
        self.self_state_pub = rospy.Publisher(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped, queue_size=1)

        self.state_sub = rospy.Subscriber(f"/{self.robot_name}/state_machine_node/state", Int32, self._state_callback)
        self.left_wheel_encoder_sub = rospy.Subscriber(f"/{self.robot_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self._left_wheel_encoder_callback)
        self.right_wheel_encoder_sub = rospy.Subscriber(f"/{self.robot_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self._right_wheel_encoder_callback)

    def _left_wheel_encoder_callback(self, msg: Int32) -> None:
        self.left_ticks = msg.data
        self.self_state_pub.publish(BoolStamped(header=rospy.Header(), data=self.is_following))

    def _right_wheel_encoder_callback(self, msg: Int32) -> None:
        self.right_ticks = msg.data
        self.self_state_pub.publish(BoolStamped(header=rospy.Header(), data=self.is_following))
    
    def _state_callback(self, state_msg) -> None:
        if state_msg.data == State.LANE_FOLLOW.value:
            if not self.is_following:
                self._start_following()
        else:
            if self.is_following:
                self._stop_following()

    def _start_following(self) -> None:
        rospy.loginfo(f"[{self.node_name}] Start following the lane")
        self.lane_follower_cmd_pub.publish(BoolStamped(header=rospy.Header(), data=False))
        self.is_following = True

    def _stop_following(self) -> None:
        rospy.loginfo(f"[{self.node_name}] Stop following the lane")
        self.lane_follower_cmd_pub.publish(BoolStamped(header=rospy.Header(), data=True))
        # keep publishing zero velocity to stop the robot until encoder values are not changing in tolerance
        prev_left_ticks = self.left_ticks
        prev_right_ticks = self.right_ticks
        start_time = rospy.get_time()
        while True:
            current_time = rospy.get_time()
            if current_time - start_time >= 0.5:  # 0.5 seconds elapsed
                if abs(self.left_ticks - prev_left_ticks) < 4 and abs(self.right_ticks - prev_right_ticks) < 4:
                    break  # Encoder values are stable
                else:
                    start_time = current_time  # Reset the timer if values changed
                    prev_left_ticks = self.left_ticks
                    prev_right_ticks = self.right_ticks
            self.wheel_twist_pub.publish(Twist2DStamped(v=0.0, omega=0.0))
            rospy.sleep(0.1)

        rospy.loginfo(f"[{self.node_name}] Robot stopped")
        self.is_following = False


if __name__ == "__main__":
    rospy.init_node("lane_following_controller_node")
    lane_following_controller_node = LaneFollowingControllerNode()
    rospy.spin()