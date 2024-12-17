#!/usr/bin/env python3
import rospy
import numpy as np
import ast
from typing import Tuple, Dict, List
import os
from planner.map_utils import Command, StreetDirection
from enum import Enum
from std_msgs.msg import Int32
from planner.srv import PlannerService, PlannerServiceRequest
from apriltag_detection.msg import ApriltagMsg
from turning.srv import TurnService, TurnServiceRequest
from turning.turn_service import TurnDirection
from duckietown_msgs.msg import BoolStamped

class State(Enum):
    WAIT_FOR_PLAN = 1
    LANE_FOLLOW = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    TURN_U = 5
    STOP = 6
    OBSTACLE_AVOIDANCE = 7

TAG_STOP_DIST = 0.3

class StateMachineNode:
    node_name: str
    robot_name: str

    state: State

    plan: List[Command]
    crossings_already_passed: List[int]

    closest_tag_id: int
    closest_tag_dist: float
    goal_tag_id: int
    obstacle_detected: bool

    state_pub: rospy.Publisher
    apriltag_sub: rospy.Subscriber

    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        self.closest_tag_dist = None
        self.closest_tag_id = None
        self.goal_tag_id = rospy.get_param(f"/{self.robot_name}/goal_tag_id", None)
        if self.goal_tag_id is None:
            raise ValueError(f"/{self.robot_name}/goal_tag_id is not set.")
        rospy.loginfo(f"[{self.node_name}] Goal tag id: %s", self.goal_tag_id)
        self.goal_tag_id = int(self.goal_tag_id)
        self.obstacle_detected = False

        self.state = State.WAIT_FOR_PLAN

        self.plan = []
        self.crossings_already_passed = []

        self.state_pub = rospy.Publisher(f"/{self.robot_name}/state_machine_node/state", Int32, queue_size=1)

        self.apriltag_sub = rospy.Subscriber(f"/{self.robot_name}/apriltag_detection_node/tag_info", ApriltagMsg, self._apriltag_cb, queue_size=1)

        self.obstacle_detection_sub = rospy.Subscriber(f"/{self.robot_name}/obstacle_detection_node/obstacle_detected", BoolStamped, self._obstacle_detection_cb, queue_size=1)


        # wait for all other services and messages to be ready [plan_service, turn_service, apriltag_detection, lane_following_controller, TODO: obstacle_detection]
        rospy.loginfo(f"[{self.node_name}] Waiting for planner service...")
        rospy.wait_for_service(f"/{self.robot_name}/planner_service")
        rospy.loginfo(f"[{self.node_name}] Waiting for turn service...")
        rospy.wait_for_service(f"/{self.robot_name}/turn_service")
        rospy.loginfo(f"[{self.node_name}] Waiting for apriltag detection message...")
        rospy.wait_for_message(f"/{self.robot_name}/apriltag_detection_node/tag_info", ApriltagMsg)
        rospy.loginfo(f"[{self.node_name}] Waiting for lane_following controller signal...")
        rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
        rospy.loginfo(f"[{self.node_name}] Waiting for obstacle_detection signal...")
        rospy.wait_for_message(f"/{self.robot_name}/obstacle_detection_node/obstacle_detected", BoolStamped)
        rospy.loginfo(f"[{self.node_name}] All services and messages are ready.")

        self._begin_state_machine()

    def _apriltag_cb(self, msg: ApriltagMsg) -> None:
        # -1 means no tag detected
        if msg.tag_id == -1:
            self.closest_tag_dist = None
            self.closest_tag_id = None
        else:
            self.closest_tag_id = msg.tag_id
            self.closest_tag_dist = msg.tag_pose.pose.position.z

    
    def _obstacle_detection_cb(self, msg: BoolStamped) -> None:
        # True means obstacle detected
        self.obstacle_detected = msg.data

    def _begin_state_machine(self) -> None:
        rospy.loginfo(f"[{self.node_name}] Starting state machine...")

        while not rospy.is_shutdown():
            self.state_pub.publish(Int32(data=self.state.value))
            if self.state == State.WAIT_FOR_PLAN:
                # rospy.loginfo(f"[{self.node_name}] State: {self.state.name}")
                planner_service = rospy.ServiceProxy(f"/{self.robot_name}/planner_service", PlannerService)
                request = PlannerServiceRequest()
                start_grid_coords_dir = rospy.get_param("~start_grid_coords_dir", None)
                goal_grid_coords_dir = rospy.get_param("~goal_grid_coords_dir", None)
                if start_grid_coords_dir is None or goal_grid_coords_dir is None:
                    raise ValueError("Start or goal grid coordinates are not set.")
                # separate the string into a list of strings by space
                start_grid_coords_dir = ast.literal_eval(start_grid_coords_dir)
                goal_grid_coords_dir = ast.literal_eval(goal_grid_coords_dir)
                # convert the third entry to StreetDirection enum
                request.start_grid_coords_dir[0] = start_grid_coords_dir[0]
                request.start_grid_coords_dir[1] = start_grid_coords_dir[1]
                request.start_grid_coords_dir[2] = StreetDirection[start_grid_coords_dir[2]].value
                request.goal_grid_coords_dir[0] = goal_grid_coords_dir[0]
                request.goal_grid_coords_dir[1] = goal_grid_coords_dir[1]
                request.goal_grid_coords_dir[2] = StreetDirection[goal_grid_coords_dir[2]].value
                rospy.loginfo(f"[{self.node_name}] Requesting plan from planner service...")

                response = planner_service(request)
                self.plan = [Command(cmd) for cmd in response.cmd_list]
                # we do not allow starting at crossing but it's possible that we are right in front of one
                if self.closest_tag_dist and self.closest_tag_dist < TAG_STOP_DIST:
                    if self.closest_tag_id != self.goal_tag_id:
                        if self.plan[self.closest_tag_id] == Command.FORWARD:
                            self.state = State.LANE_FOLLOW
                        elif self.plan[self.closest_tag_id] == Command.LEFT:
                            self.state = State.TURN_LEFT
                        elif self.plan[self.closest_tag_id] == Command.RIGHT:
                            self.state = State.TURN_RIGHT
                        elif self.plan[self.closest_tag_id] == Command.UTURN:
                            self.state = State.TURN_U
                        else:
                            rospy.logwarn(f"[{self.node_name}] Invalid command at crossing {self.closest_tag_id}: {self.plan[self.closest_tag_id]}")
                        self.crossings_already_passed.append(self.closest_tag_id)
                    else:
                        self.state = State.STOP
                else:
                    self.state = State.LANE_FOLLOW
                continue

            elif self.state == State.LANE_FOLLOW:
                # rospy.loginfo(f"[{self.node_name}] State: LANE_FOLLOW")
                if self.closest_tag_id and self.closest_tag_id == self.goal_tag_id and self.closest_tag_dist < TAG_STOP_DIST:
                    # print this time to check how much time we need to stop
                    self.time = rospy.get_time()
                    self.state = State.STOP
                    continue

                # TODO: check obstacles
                if self.obstacle_detected is True:
                    self.state = State.OBSTACLE_AVOIDANCE
                    continue
                
                
                # check if we are at a crossing
                if self.closest_tag_dist and self.closest_tag_dist < TAG_STOP_DIST:
                    if self.closest_tag_id in self.crossings_already_passed:
                        rospy.logwarn(f"[{self.node_name}] Already passed crossing {self.closest_tag_id} once, keep lane following.")
                        continue
                    if self.plan[self.closest_tag_id] == Command.FORWARD:
                        self.state = State.LANE_FOLLOW
                    elif self.plan[self.closest_tag_id] == Command.LEFT:
                        self.state = State.TURN_LEFT
                    elif self.plan[self.closest_tag_id] == Command.RIGHT:
                        self.state = State.TURN_RIGHT
                    elif self.plan[self.closest_tag_id] == Command.UTURN:
                        self.state = State.TURN_U
                    else:
                        rospy.logwarn(f"[{self.node_name}] Invalid command at crossing {self.closest_tag_id}: {self.plan[self.closest_tag_id]}")
                    self.crossings_already_passed.append(self.closest_tag_id)
                
            elif self.state == State.TURN_LEFT:
                # make sure lane following is stopped
                lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                while lane_follow_state.data:
                    rospy.loginfo(f"[{self.node_name}] Waiting for lane following to stop...")
                    lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                # rospy.loginfo(f"[{self.node_name}] State: TURN_LEFT")
                turn_service = rospy.ServiceProxy(f"/{self.robot_name}/turn_service", TurnService)
                request = TurnServiceRequest()
                request.direction = TurnDirection.LEFT.value
                rospy.loginfo(f"[{self.node_name}] Requesting left turn from turn service...")
                response = turn_service(request)
                self.state = State.LANE_FOLLOW

            elif self.state == State.TURN_RIGHT:
                # make sure lane following is stopped
                lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                while lane_follow_state.data:
                    rospy.loginfo(f"[{self.node_name}] Waiting for lane following to stop...")
                    lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                # rospy.loginfo(f"[{self.node_name}] State: TURN_RIGHT")
                turn_service = rospy.ServiceProxy(f"/{self.robot_name}/turn_service", TurnService)
                request = TurnServiceRequest()
                request.direction = TurnDirection.RIGHT.value
                rospy.loginfo(f"[{self.node_name}] Requesting right turn from turn service...")
                response = turn_service(request)
                self.state = State.LANE_FOLLOW

            elif self.state == State.TURN_U:
                # make sure lane following is stopped
                lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                while lane_follow_state.data:
                    rospy.loginfo(f"[{self.node_name}] Waiting for lane following to stop...")
                    lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                # rospy.loginfo(f"[{self.node_name}] State: TURN_U")
                turn_service = rospy.ServiceProxy(f"/{self.robot_name}/turn_service", TurnService)
                request = TurnServiceRequest()
                request.direction = TurnDirection.U.value
                rospy.loginfo(f"[{self.node_name}] Requesting U-turn from turn service...")
                response = turn_service(request)
                self.state = State.LANE_FOLLOW
            
            elif self.state == State.STOP:
                lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                while lane_follow_state.data:
                    rospy.loginfo(f"[{self.node_name}] Waiting for lane following to stop...")
                    lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                rospy.loginfo(f"{rospy.get_time() - self.time} sec needed to stop")
                # let user know that we have reached the goal and use ctrl+c to stop the node
                rospy.loginfo(f"[{self.node_name}] Goal reached! Use ctrl+c to stop the node.")
                break

            elif self.state == State.OBSTACLE_AVOIDANCE:
                lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                while lane_follow_state.data:
                    rospy.loginfo(f"[{self.node_name}] Obstacle! Waiting for lane following to stop...")
                    lane_follow_state = rospy.wait_for_message(f"/{self.robot_name}/lane_following_controller_node/state", BoolStamped)
                turn_service = rospy.ServiceProxy(f"/{self.robot_name}/turn_service", TurnService)
                request = TurnServiceRequest()
                request.direction = TurnDirection.STOP.value
                rospy.loginfo(f"[{self.node_name}] Requesting stop from turn service...")
                response = turn_service(request)
                while self.obstacle_detected:
                    rospy.sleep(0.1)
                self.state = State.LANE_FOLLOW
            else:
                raise ValueError(f"[{self.node_name}] Invalid state: {self.state}")
    

if __name__ == "__main__":
    rospy.init_node("state_machine_node")
    state_machine_node = StateMachineNode()
    rospy.spin()