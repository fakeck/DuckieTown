#!/usr/bin/env python3
from map_utils import MapGraph
import rospy
import numpy as np
import yaml
from typing import Tuple, Dict, List
import os
from planner.msg import PlannerCmdMsg


class PlannerNode:
    node_name: str
    robot_name: str
    
    map_graph: MapGraph
    
    state_sub: rospy.Subscriber

    cmd_pub: rospy.Publisher

    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        yaml_path = rospy.get_param("~map_yaml_path")
        if yaml_path is None:
            raise ValueError("~map_yaml_path is not set")
        rospy.loginfo(f"[{self.node_name}] Map YAML path: %s", yaml_path)
        self.map_graph = MapGraph(yaml_path)
        
        # self.state_sub = rospy.Subscriber(f"/{self.robot_name}/state", StateMsg, self._state_callback) TODO: wait for state message
        self.cmd_pub = rospy.Publisher(f"/{self.robot_name}/planner_node/cmd", PlannerCmdMsg, queue_size=10)
    

    def _state_callback(self, state_msg) -> None:
        if state_msg.data == 0: # TODO change to state enum
            rospy.loginfo(f"[{self.node_name}] State: Waiting for plan")
            start_r_c_dir = state_msg.start_r_c_dir # TODO List[int, int, StreetDirection]
            goal_r_c = state_msg.goal_r_c # TODO List[int, int, StreetDirection]
            shortest_path = self.map_graph.shortest_path(tuple(start_r_c_dir), tuple(goal_r_c))
            cmd_list = self.map_graph.reduce_to_crossing_cmd(shortest_path)
            msg = PlannerCmdMsg()
            msg.start_grid_coords_dir = start_r_c_dir
            msg.goal_grid_coords = goal_r_c
            msg.cmd_list = cmd_list
            self.cmd_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("planner_node")
    planner_node = PlannerNode()
    rospy.spin()