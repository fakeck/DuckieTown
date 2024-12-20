#!/usr/bin/env python3

# Map: stores a map from a YAML file
# MapGraph: encodes a graph representation of the map and provides methods to find the shortest path.
from enum import Enum
from typing import Dict, List, Tuple
from enum import Enum
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import yaml

class Command(Enum):
    START = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    UTURN = 4
    STOP = 5

class NodeType(Enum):
    CROSSING = 1
    STREET = 2
    CORNER = 3

class StreetDirection(Enum):
    # For street
    N = 1
    S = 2
    E = 3
    W = 4
    # For crossing
    X = 5
    # For corner
    NE = 6
    EN = 7
    NW = 8
    WN = 9
    SE = 10
    ES = 11
    SW = 12
    WS = 13

class Node:
    r: int # row idx
    c: int # col idx
    type: NodeType
    direction: StreetDirection

    def __init__(self, r: int, c: int, direction: StreetDirection):
        self.r = r
        self.c = c
        self.direction = direction
        self.type = None


class StreetNode(Node):
    def __init__(self, r: int, c: int, direction: StreetDirection):
        super().__init__(r, c, direction)
        self.type = NodeType.STREET

class CrossingNode(Node):
    tag_id: int

    def __init__(self, r: int, c: int, tag_id: int):
        super().__init__(r, c, StreetDirection.X)
        self.tag_id = tag_id
        self.type = NodeType.CROSSING

class CornerNode(Node):
    def __init__(self, r: int, c: int, direction: StreetDirection):
        super().__init__(r, c, direction)
        self.direction = direction
        self.type = NodeType.CORNER

class Map:
    map_2d: List[List[int]] # 2D map [[row][row][row]...]
    crossing_tag_id: Dict[int, List[int]] # Tag id [i] to crossing coordinates [row, col]
    corner_coords: List[List[int]] # Corner coordinates [[row, col]]

    def __init__(self, map_yaml_path):
        self.map_2d, self.crossing_tag_id = self.__read_map(map_yaml_path)
        self.corner_coords = self.__get_corner_coords()

    def __read_map(self, yaml_path) -> Tuple[np.ndarray, Dict[int, list]]:
        with open(yaml_path, "r") as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            map = np.array(data["map"])
            tags = data["tags"]
        return map, tags

    def __get_corner_coords(self):
        """ return a list of all corner coordinates in the map """
        coner_coords = []
        for r in range(len(self.map_2d)):
            for c in range(len(self.map_2d[0])):
                if self.map_2d[r][c] == 1 or [r, c] in self.crossing_tag_id.values():
                    continue
                # if the cell is free, and it's not a crossing, and it's free on both N/S and E/W directions
                # N/S check
                if r > 0 and self.map_2d[r-1][c] == 0 or r < len(self.map_2d)-1 and self.map_2d[r+1][c] == 0:
                    # E/W check
                    if c > 0 and self.map_2d[r][c-1] == 0 or c < len(self.map_2d[0])-1 and self.map_2d[r][c+1] == 0:
                        coner_coords.append([r, c])
        return coner_coords


class MapGraph:
    map: Map
    nx_graph: nx.DiGraph
    __coord_node_dict: Dict[Tuple[int, int, StreetDirection], Node] # (r, c, dir) -> node
    __node_neighbors_dict: Dict[Node, List[Node]] # node -> neighbors

    def __init__(self, map_yaml_path) -> None:
        self.map = Map(map_yaml_path)
        self.__coord_node_dict = self.__init_node()
        self.__node_neighbors_dict = self.__init_neighbors()
        self.nx_graph = self.__build_nx_graph()

    def __init_node(self) -> Dict[Tuple[int, int, StreetDirection], Node]:
        __coord_node_dict = {}
        for r, c in [(r, c) for r in range(len(self.map.map_2d)) for c in range(len(self.map.map_2d[0]))]:
            if [r, c] in self.map.crossing_tag_id.values(): # crossing
                node = CrossingNode(r, c, list(self.map.crossing_tag_id.keys())[list(self.map.crossing_tag_id.values()).index([r, c])])
                __coord_node_dict[(r, c, StreetDirection.X)] = node
            elif [r, c] in self.map.corner_coords: # corner
                # if it's free on NE
                if (r > 0 and c < len(self.map.map_2d[0])-1 and self.map.map_2d[r-1][c] == 0 and self.map.map_2d[r][c+1] == 0):
                    __coord_node_dict[(r, c, StreetDirection.WN)] = CornerNode(r, c, StreetDirection.WN)
                    __coord_node_dict[(r, c, StreetDirection.SE)] = CornerNode(r, c, StreetDirection.SE)
                # if it's free on SW
                elif (r < len(self.map.map_2d)-1 and c > 0 and self.map.map_2d[r+1][c] == 0 and self.map.map_2d[r][c-1] == 0):
                    __coord_node_dict[(r, c, StreetDirection.NW)] = CornerNode(r, c, StreetDirection.NW)
                    __coord_node_dict[(r, c, StreetDirection.ES)] = CornerNode(r, c, StreetDirection.ES)
                # if it's free on NW
                elif (r > 0 and c > 0 and self.map.map_2d[r-1][c] == 0 and self.map.map_2d[r][c-1] == 0):
                    __coord_node_dict[(r, c, StreetDirection.EN)] = CornerNode(r, c, StreetDirection.EN)
                    __coord_node_dict[(r, c, StreetDirection.SW)] = CornerNode(r, c, StreetDirection.SW)
                # if it's free on SE
                elif (r < len(self.map.map_2d)-1 and c < len(self.map.map_2d[0])-1 and self.map.map_2d[r+1][c] == 0 and self.map.map_2d[r][c+1] == 0):
                    __coord_node_dict[(r, c, StreetDirection.WS)] = CornerNode(r, c, StreetDirection.WS)
                    __coord_node_dict[(r, c, StreetDirection.NE)] = CornerNode(r, c, StreetDirection.NE)

            elif self.map.map_2d[r][c] == 0: # street
                if c > 0 and self.map.map_2d[r][c-1] == 0 or c < len(self.map.map_2d[0])-1 and self.map.map_2d[r][c+1] == 0:
                    __coord_node_dict[(r, c, StreetDirection.W)] = StreetNode(r, c, StreetDirection.W)
                    __coord_node_dict[(r, c, StreetDirection.E)] = StreetNode(r, c, StreetDirection.E)
                elif r > 0 and self.map.map_2d[r-1][c] == 0 or r < len(self.map.map_2d)-1 and self.map.map_2d[r+1][c] == 0:
                    __coord_node_dict[(r, c, StreetDirection.S)] = StreetNode(r, c, StreetDirection.S)
                    __coord_node_dict[(r, c, StreetDirection.N)] = StreetNode(r, c, StreetDirection.N)
                   
        return __coord_node_dict

    def __init_neighbors(self) -> Dict[Node, List[Node]]:
        __node_neighbors_dict = {}
        for node in self.__coord_node_dict.values():
            neighbors = self.__get_neighbors(node)
            __node_neighbors_dict[node] = neighbors
        return __node_neighbors_dict

    def __get_neighbors(self, node: Node) -> List[Node]:
        """ return all reachable neighbors of the node """
        neighbors = []
        type = node.type
        if type == NodeType.CROSSING:
            for direction in StreetDirection:
                neighbor = self.__get_neighbor(node, direction)
                if neighbor:
                    neighbors.append(neighbor)
        elif type == NodeType.STREET:
            neighbor = self.__get_neighbor(node, node.direction)
            if neighbor:
                neighbors.append(neighbor)
        elif type == NodeType.CORNER:
            r = node.r
            c = node.c
            if node.direction == StreetDirection.NE or node.direction == StreetDirection.EN:
                neighbor = self.__get_neighbor(node, StreetDirection.N)
                if neighbor:
                    neighbors.append(neighbor)
                neighbor = self.__get_neighbor(node, StreetDirection.E)
                if neighbor:
                    neighbors.append(neighbor)
            if node.direction == StreetDirection.NW or node.direction == StreetDirection.WN:
                neighbor = self.__get_neighbor(node, StreetDirection.N)
                if neighbor:
                    neighbors.append(neighbor)
                neighbor = self.__get_neighbor(node, StreetDirection.W)
                if neighbor:
                    neighbors.append(neighbor)
            if node.direction == StreetDirection.SE or node.direction == StreetDirection.ES:
                neighbor = self.__get_neighbor(node, StreetDirection.S)
                if neighbor:
                    neighbors.append(neighbor)
                neighbor = self.__get_neighbor(node, StreetDirection.E)
                if neighbor:
                    neighbors.append(neighbor)
            if node.direction == StreetDirection.SW or node.direction == StreetDirection.WS:
                neighbor = self.__get_neighbor(node, StreetDirection.S)
                if neighbor:
                    neighbors.append(neighbor)
                neighbor = self.__get_neighbor(node, StreetDirection.W)
                if neighbor:
                    neighbors.append(neighbor)
        return neighbors

    def __get_neighbor(self, node: Node, direction: StreetDirection) -> Node:
        """ return the neighbor node at the given direction if it's reachable, otherwise return None """
        neighbor = None
        r = node.r
        c = node.c
        if direction == StreetDirection.N and r > 0 and self.map.map_2d[r-1][c] == 0:
            neighbor_r = r - 1
            neighbor_c = c
            # check type of neighbor
            if [neighbor_r, neighbor_c] in self.map.crossing_tag_id.values():
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.X)]
            elif [neighbor_r, neighbor_c] in self.map.corner_coords:
                if c < len(self.map.map_2d[0])-1 and self.map.map_2d[neighbor_r][c+1] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.NE)]
                elif c > 0 and self.map.map_2d[neighbor_r][c-1] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.NW)]
            else:
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.N)]
        elif direction == StreetDirection.S and r < len(self.map.map_2d)-1 and self.map.map_2d[r+1][c] == 0:
            neighbor_r = r + 1
            neighbor_c = c
            # check type of neighbor
            if [neighbor_r, neighbor_c] in self.map.crossing_tag_id.values():
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.X)]
            elif [neighbor_r, neighbor_c] in self.map.corner_coords:
                if c > 0 and self.map.map_2d[neighbor_r][c-1] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.SW)]
                elif c < len(self.map.map_2d[0])-1 and self.map.map_2d[neighbor_r][c+1] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.SE)]
            else:
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.S)]
        elif direction == StreetDirection.E and c < len(self.map.map_2d[0])-1 and self.map.map_2d[r][c+1] == 0:
            neighbor_r = r
            neighbor_c = c + 1
            # check type of neighbor
            if [neighbor_r, neighbor_c] in self.map.crossing_tag_id.values():
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.X)]
            elif [neighbor_r, neighbor_c] in self.map.corner_coords:
                if r < len(self.map.map_2d)-1 and self.map.map_2d[r+1][neighbor_c] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.ES)]
                elif r > 0 and self.map.map_2d[r-1][neighbor_c] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.EN)]
            else:
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.E)]
        elif direction == StreetDirection.W and c > 0 and self.map.map_2d[r][c-1] == 0:
            neighbor_r = r
            neighbor_c = c - 1
            # check type of neighbor
            if [neighbor_r, neighbor_c] in self.map.crossing_tag_id.values():
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.X)]
            elif [neighbor_r, neighbor_c] in self.map.corner_coords:
                if r < len(self.map.map_2d)-1 and self.map.map_2d[r+1][neighbor_c] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.WS)]
                elif r > 0 and self.map.map_2d[r-1][neighbor_c] == 0:
                    neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.WN)]
            else:
                neighbor = self.__coord_node_dict[(neighbor_r, neighbor_c, StreetDirection.W)]
        return neighbor
    
    def __get_node_weight(self, node: Node) -> float:
        """Assign weights to node types."""
        if node.type == NodeType.CROSSING:
            return 2
        elif node.type == NodeType.STREET:
            return 1
        elif node.type == NodeType.CORNER:
            return 1.5
        return float('inf')  # For safety
    
    def __get_edge_weight(self, parent: Node, child: Node) -> float:
        """Assign weights to edges."""
        # if child is reachable from parent
        if child in self.__node_neighbors_dict[parent]:
            return 0 # TODO: depending on the direction change, the cost can be different for different edges
        return float('inf')  # For safety
    
    def __build_nx_graph(self):
        """Build a networkx graph from the map representation."""
        G = nx.DiGraph()
        
        # Add nodes with weights (optional)
        for coord, node in self.__coord_node_dict.items():
            G.add_node(coord, type=node.type, weight=self.__get_node_weight(node))
        
        # Add edges
        for node, neighbors in self.__node_neighbors_dict.items():
            for neighbor in neighbors:
                weight = self.__get_edge_weight(node, neighbor)
                G.add_edge(
                    (node.r, node.c, getattr(node, 'direction', None)),
                    (neighbor.r, neighbor.c, getattr(neighbor, 'direction', None)),
                    weight=weight
                )
        return G

    def shortest_path(self, start: Tuple[int, int, StreetDirection], end: Tuple[int, int, StreetDirection]) -> List[Tuple[int, int, StreetDirection]]:
        """Find the shortest path using networkx's Dijkstra's algorithm."""
        # You are not allowed to start or stop at a crossing
        assert start[2] != StreetDirection.X and end[2] != StreetDirection.X, "Start and end nodes cannot be crossings."
        G = self.nx_graph
        try:
            path = nx.shortest_path(G, source=start, target=end, weight='weight')
        except nx.NetworkXNoPath:
            path = []
            print("No path found.")
        return path

    def visualize_path(self, path, file_path=None):
        """Visualize the shortest path directly using networkx."""
        # Prepare the networkx graph
        G = nx.DiGraph()
        plt.figure(figsize=(5, 10))
        
        # Offset values for different directions to avoid overlap
        direction_offsets = {
            StreetDirection.N: (0.2, 0),
            StreetDirection.S: (-0.2, 0),
            StreetDirection.E: (0, -0.1),
            StreetDirection.W: (0, 0.1),
            StreetDirection.NE: (0.1, -0.1),
            StreetDirection.EN: (0.1, -0.1),
            StreetDirection.NW: (0.1, 0.1),
            StreetDirection.WN: (0.1, 0.1),
            StreetDirection.SE: (-0.1, -0.1),
            StreetDirection.ES: (-0.1, -0.1),
            StreetDirection.SW: (-0.1, 0.1),
            StreetDirection.WS: (-0.1, 0.1),
            StreetDirection.X: (0, 0)  # For crossings
        }

        # Add nodes and edges (same as visualize_graph)
        for (r, c, direction), node in self.__coord_node_dict.items():
            label = f"{r},{c},{direction.name}"
            G.add_node((r, c, direction), label=label, type=node.type.name)
        
        for node, neighbors in self.__node_neighbors_dict.items():
            for neighbor in neighbors:
                G.add_edge((node.r, node.c, getattr(node, 'direction', None)),
                        (neighbor.r, neighbor.c, getattr(neighbor, 'direction', None)))

        # Prepare positions with offsets
        pos = {}
        for (r, c, direction) in G.nodes:
            base_pos = (c, -r )
            offset = direction_offsets.get(direction, (0, 0))
            pos[(r, c, direction)] = (base_pos[0] + offset[0], base_pos[1] + offset[1])
        
        # Draw the base graph
        plt.figure(figsize=(5, 10))
        node_colors = []
        for node in G.nodes:
            if 'X' in node[2].name:  # Check if it's a crossing (assuming 'X' denotes a crossing)
                node_colors.append('#66CCFF')
            elif len(node[2].name) > 1:  # Check if it's a corner
                node_colors.append('#99FFFF')
            else:
                node_colors.append('#CCFFFF')
        # Draw nodes
        nx.draw(G, pos, with_labels=False, node_size=500, node_color=node_colors)
        labels = nx.get_node_attributes(G, 'label')
        nx.draw_networkx_labels(G, pos, labels, font_size=6)
        nx.draw_networkx_edges(G, pos, arrows=True, arrowstyle='->', alpha=0.6)

        # Draw the weight labels below each node label
        node_weights = {node: self.__get_node_weight(self.__coord_node_dict[node]) for node in G.nodes}
        weight_labels = {node: f"cost: {node_weights[node]}" for node in G.nodes}
        weight_positions = {node: (pos[node][0], pos[node][1] - 0.1) for node in G.nodes}
        nx.draw_networkx_labels(G, weight_positions, weight_labels, font_size=6, font_color='black')

        # Highlight the path
        path_edges = []
        path_nodes = set()
        total_cost = 0  # Initialize total cost
        for i in range(len(path) - 1):
            # Add the edge between consecutive nodes in the path
            path_edges.append((path[i], path[i+1]))
            path_nodes.add(path[i])
            path_nodes.add(path[i+1])
            
            # Sum the cost of the edge (you may also need to sum node weights depending on the logic)
            total_cost += self.__get_node_weight(self.__coord_node_dict[path[i]])

        node_colors = []
        for node in path_nodes:
            # if start or end node, color it differently
            if node == path[0]:
                node_colors.append('#FF3333')
            elif node == path[-1]:
                node_colors.append('#00CC33')
            elif 'X' in node[2].name:  # Check if it's a crossing
                node_colors.append('#FFCC00')
            elif len(node[2].name) > 1:  # Check if it's a corner
                node_colors.append('#FFCC33')
            else:
                node_colors.append('#FFCC66')
        # Draw the path nodes in a different color (e.g., red)
        nx.draw_networkx_nodes(G, pos, nodelist=path_nodes, node_size=700, node_color=node_colors)

        # Draw the path edges in a different style (e.g., thicker lines)
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=2, edge_color='#FFCC00', alpha=0.8)
        plt.text(0.85, 0.79, f"Total Cost: {total_cost}", ha='right', va='top', transform=plt.gca().transAxes,
             fontsize=8, color='black', bbox=dict(facecolor='white', edgecolor='none', alpha=0.5))
        plt.text(0.85, 0.85, "Red: Start", ha='right', va='top', transform=plt.gca().transAxes, fontsize=8, color='red')
        plt.text(0.85, 0.83, "Green: Goal", ha='right', va='top', transform=plt.gca().transAxes, fontsize=8, color='green')
        plt.text(0.85, 0.81, "Orange: Path", ha='right', va='top', transform=plt.gca().transAxes, fontsize=8, color='orange')
        # Save or show the visualization
        plt.title("Map Graph with Shortest Path")
        if file_path:
            plt.savefig(file_path)
        else:
            plt.show()

    def reduce_to_crossing_cmd(self, path: List[Tuple[int, int, StreetDirection]]) -> List[Command]:
        """Reduce the path to a list of crossing commands."""
        # retrieve the path and output a list [START, crossing_tagid1_cmd, crossing_id2_cmd, crossing_id3_cmd, crossing_id4_cmd, crossing_id5_cmd, STOP]
        # initialize as number of tag ids + 2
        crossing_cmds = [Command.STOP] * (len(self.map.crossing_tag_id) + 2)
        crossing_cmds[0] = Command.START
        for i in range(1, len(path)-1):
            node = self.__coord_node_dict[path[i]]
            if node.type == NodeType.CROSSING:
                tag_id = node.tag_id
                prev_node = path[i - 1]
                next_node = path[i + 1]
                if (prev_node[2] == StreetDirection.N and next_node[2] == StreetDirection.S
                    or prev_node[2] == StreetDirection.S and next_node[2] == StreetDirection.N
                    or prev_node[2] == StreetDirection.E and next_node[2] == StreetDirection.W
                    or prev_node[2] == StreetDirection.W and next_node[2] == StreetDirection.E):
                    crossing_cmds[tag_id] = Command.UTURN
                elif prev_node[2].name[-1] == next_node[2].name[0]:
                    crossing_cmds[tag_id] = Command.FORWARD
                elif prev_node[2].name[-1] == 'N':
                    crossing_cmds[tag_id] = Command.RIGHT if next_node[2].name[0] == 'E' else Command.LEFT
                elif prev_node[2].name[-1] == 'S':
                    crossing_cmds[tag_id] = Command.LEFT if next_node[2].name[0] == 'E' else Command.RIGHT
                elif prev_node[2].name[-1] == 'E':
                    crossing_cmds[tag_id] = Command.LEFT if next_node[2].name[0] == 'N' else Command.RIGHT
                elif prev_node[2].name[-1] == 'W':
                    crossing_cmds[tag_id] = Command.RIGHT if next_node[2].name[0] == 'N' else Command.LEFT
        return crossing_cmds
            

if __name__ == "__main__":
    map_graph = MapGraph("/code/catkin_ws/src/user_code/project/planner/config/map.yaml")
    start = (0, 0, StreetDirection.WS)

    for end in [(7, 3, StreetDirection.W), (2, 2, StreetDirection.N), (7, 5, StreetDirection.SW)]:
        path = map_graph.shortest_path(start, end)
        # map_graph.visualize_path(path, "/code/catkin_ws/src/user_code/project/planner/config/map_graph.png")
        print(map_graph.reduce_to_crossing_cmd(path))
