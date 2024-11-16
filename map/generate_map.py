#!/usr/bin/env python3
import yaml

# -> : y-axis
# |
# v : x-axis
# 0: empty
# 1: obstacle
map_array = [[0, 0, 0, 1, 1, 1],
             [0, 1, 0, 1, 1, 1],
             [0, 1, 0, 1, 1, 1],
             [0, 0, 0, 0, 0, 1],
             [0, 1, 0, 1, 0, 0],
             [0, 0, 0, 1, 1, 0],
             [0, 1, 0, 1, 1, 0],
             [0, 0, 0, 0, 0, 0]
            ]

grid_coords_tag_id = [
    # TODO: Add at the grid coordinates the placed AprilTag's id
]

yaml_path = "/code/catkin_ws/src/user_code/project/map/map.yaml"
with open(yaml_path, "w") as file:
    yaml.dump({"map": map_array}, file)

# TODO: Write the grid_coords_tag_id to a YAML file