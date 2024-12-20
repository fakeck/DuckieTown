#!/usr/bin/env python3

# This node provides the ROS service to turn the Duckiebot in different directions.
# rosrun turning turn_service.py
# rosservice call /<robot_name>/turn_service "direction: 1"  # 1: LEFT, 2: RIGHT, 3: U, 4: STOP
import os
import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
from enum import Enum
from turning.srv import TurnService

class TurnDirection(Enum):
    LEFT = 1
    RIGHT = 2
    U = 3
    STOP = 4

# Parameters left
VELOCITY_LEFT = 0.3 # Linear velocity in m/s
RADIUS_LEFT = 0.18
TURN_ANGLE_LEFT = 90 # The angle to turn in degrees )
TURN_TIME_LEFT = 2
# Parameters right
VELOCITY_RIGHT = 0.3 # Linear velocity in m/s
RADIUS_RIGHT = -0.1 # Keep it negative to turn right
TURN_ANGLE_RIGHT = 90 # The angle to turn in degrees
TURN_TIME_RIGHT = 1.2
# Parameters U
VELOCITY_U = 0.3 # Linear velocity in m/s
RADIUS_U = 0.07
TURN_ANGLE_U = 180 # The angle to turn in degrees
TURN_TIME_U = 1.7


class TurnServer():
    node_name: str
    robot_name: str

    left_ticks: int
    right_ticks: int

    wheel_twist_pub: rospy.Publisher
    turn_service: rospy.Service

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        # Get robot name from the environment variable
        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        self.left_ticks = 0
        self.right_ticks = 0
        
        self.wheel_twist_pub = rospy.Publisher(f"/{self.robot_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        self.left_wheel_encoder_sub = rospy.Subscriber(f"/{self.robot_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self._left_wheel_encoder_callback)
        self.right_wheel_encoder_sub = rospy.Subscriber(f"/{self.robot_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self._right_wheel_encoder_callback)
        
        self.turn_service = rospy.Service(f"/{self.robot_name}/turn_service", TurnService, self._turn_service_callback)

    def _left_wheel_encoder_callback(self, msg: WheelEncoderStamped) -> None:
        self.left_ticks = msg.data

    def _right_wheel_encoder_callback(self, msg: WheelEncoderStamped) -> None:
        self.right_ticks = msg.data

    
    def _turn_service_callback(self, request):
        dir = TurnDirection(request.direction)
        start_time = rospy.get_time()

        if dir == TurnDirection.LEFT:
            while rospy.get_time() - start_time < TURN_TIME_LEFT:
                msg = Twist2DStamped(v=VELOCITY_LEFT, omega=VELOCITY_LEFT/RADIUS_LEFT)
                self.wheel_twist_pub.publish(msg)
                rospy.sleep(0.1)
            
        elif dir == TurnDirection.U:
            while rospy.get_time() - start_time < TURN_TIME_U:
                msg = Twist2DStamped(v=VELOCITY_U, omega=VELOCITY_U/RADIUS_U)
                self.wheel_twist_pub.publish(msg)
                rospy.sleep(0.1)

        elif dir == TurnDirection.RIGHT:
            while rospy.get_time() - start_time < TURN_TIME_RIGHT:
                msg = Twist2DStamped(v=VELOCITY_RIGHT, omega=VELOCITY_RIGHT/RADIUS_RIGHT)
                self.wheel_twist_pub.publish(msg)
                rospy.sleep(0.1)

        elif dir == TurnDirection.STOP:
            pass

        else:
            raise ValueError(f"[{self.node_name}] Invalid turn direction: {dir}")
        
        self.stop_robot()
        rospy.loginfo(f"[{self.node_name}] Turned {dir} successfully.")
        return True


    def stop_robot(self):
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
        


        
if __name__ == '__main__':
    rospy.init_node('turn_server')
    turn_server = TurnServer()
    rospy.spin()
    