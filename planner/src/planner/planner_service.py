#!/usr/bin/env python3

# This service receives a start and goal grid with heading direction.
# Dijkstra's algorithm is used to find the shortest path between the start and goal.
# The path is reduced to a series of optimal command at each crossing.
# e.g. [PLACEHOLDER, LEFT, RIGHT, FORWARD, LEFT, FORWARD, RIGHT, PLACEHOLDER]
from map_utils import MapGraph, Command, StreetDirection
import rospy
import numpy as np
from typing import Tuple, Dict, List
import os
from planner.srv import PlannerService, PlannerServiceResponse


class PlannerServer:
    node_name: str
    robot_name: str
    map_graph: MapGraph

    planner_service: rospy.Service

    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        rospy.loginfo(f"[{self.node_name}] Initializing {self.node_name} node...")

        # Get robot name from the environment variable
        self.robot_name = os.getenv('VEHICLE_NAME', None)
        if self.robot_name is None:
            raise ValueError("$VEHICLE_NAME is not set, export it first.")
        rospy.loginfo(f"[{self.node_name}] Robot name: {self.robot_name}")

        # Load the map graph from the YAML file specified in the ROS parameter
        yaml_path = rospy.get_param("~map_yaml_path")
        if yaml_path is None:
            raise ValueError("~map_yaml_path is not set")
        rospy.loginfo(f"[{self.node_name}] Map YAML path: %s", yaml_path)
        self.map_graph = MapGraph(yaml_path)

        # Define the service
        self.planner_service = rospy.Service(f"/{self.robot_name}/planner_service", PlannerService, self._planner_service_callback)

    def _planner_service_callback(self, request) -> PlannerServiceResponse:

        # Extract start and goal coordinates and directions from the request
        start_grid_coords_dir = request.start_grid_coords_dir
        goal_grid_coords_dir = request.goal_grid_coords_dir

        start = tuple([start_grid_coords_dir[0], start_grid_coords_dir[1], StreetDirection(start_grid_coords_dir[2])])
        goal = tuple([goal_grid_coords_dir[0], goal_grid_coords_dir[1], StreetDirection(goal_grid_coords_dir[2])])

        rospy.loginfo(f"[{self.node_name}] Plan from Start: {start} to Goal: {goal}")

        # Calculate the shortest path
        shortest_path = self.map_graph.shortest_path(start, goal)

        # Reduce the path to a series of commands
        cmd_list = self.map_graph.reduce_to_crossing_cmd(shortest_path)
        rospy.loginfo(f"[{self.node_name}] Plan: {cmd_list}")

        # Create a response and fill it with the planned commands
        response = PlannerServiceResponse()
        response.start_grid_coords_dir = start_grid_coords_dir
        response.goal_grid_coords_dir = goal_grid_coords_dir
        response.cmd_list = [cmd.value for cmd in cmd_list]

        return response


if __name__ == "__main__":
    rospy.init_node("planner_server")
    planner_server = PlannerServer()
    rospy.spin()
