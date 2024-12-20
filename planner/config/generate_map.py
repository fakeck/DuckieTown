#!/usr/bin/env python3
import yaml

# Static map: 2D occupancy grid
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

# Grid coordinates with associated AprilTag IDs
grid_coords_tag_id = {
    1: [3, 0],
    2: [5, 0],
    3: [3, 2],
    4: [5, 2],
    5: [7, 2]
}

yaml_path = "/code/catkin_ws/src/user_code/project/planner/config/map.yaml"
# Write map and tag information to the YAML file
with open(yaml_path, "w") as file:
    yaml.dump({"map": map_array, "tags": grid_coords_tag_id}, file, default_flow_style=False)

print(f"Map and tag information written to {yaml_path}")