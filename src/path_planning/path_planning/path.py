
import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap



class Point():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class pathplanning(Node): 
    def __init__(self,map): #map is a class with map.grid being a dictionary with key (x,y) and value status, with 0 for clear 
        self.map=map
        self.grid=map.grid
        super().__init__("path_planning_node")

    def get_neighbors(self,pos:tuple):
            neighbors = {}
            for (dx,dy) in [(0,1), (1,0), (0,-1), (-1,0)]:  # 4-connected grid
                new_x = pos[0] + dx
                new_y = pos[1] + dy
                if self.map.get(new_x,new_y) == 0:
                    neighbors[(new_x,new_y)] = self.map.get(new_x,new_y)
            return neighbors
            

    def nav(self,coord1:tuple,coord2:tuple): ## elementary path finding algorithim for now please keep the input and output types same. 
        self.get_logger().info(f"Navigating from {coord1} to {coord2}")
        paths = [[coord1]]
        found = False
        path = []
        new_paths = []
        visited = [coord1]
        while (not found) and paths:
            for p in paths:
                neigh = self.get_neighbors(p[-1])
                for n,v in neigh.items():
                    if v != 0:
                        continue 
                    repeat = False
                    for x in visited:
                        if x[0] == n[0] and x[1] == n[1]:
                            repeat = True
                    if repeat:
                        continue
                    new_path = p + [n]
                    visited.append(n)
                    new_paths.append(new_path)
                    if n[0] == coord2[0] and n[1] == coord2[1]:
                        found = True
                        path = new_path
                        break
                if found:
                    break
            if found:
                break
            paths = new_paths
            new_paths = []
        return path
        
    def van(self,coord1:tuple,coord2:tuple,i:int):
        paths = [[coord1]]
        found = False
        path = []
        new_paths = []
        visited = [coord1]
        while (not found) and paths:
            for p in paths:
                neigh = self.get_neighbors(p[-1])
                for n,v in neigh.items():
                    if v != 0:
                        continue 
                    repeat = False
                    for x in visited:
                        if x[0] == n[0] and x[1] == n[1]:
                            repeat = True
                    if repeat:
                        continue
                    new_path = p + [n]
                    visited.append(n)
                    new_paths.append(new_path)
                    if n[0] == coord2[0] and n[1] == coord2[1]:
                        found = True
                        path = new_path
                        break
                if found:
                    break
            if found:
                break
            paths = new_paths
            if len(paths[0])>=i:
                return []
            new_paths = []
        self.get_logger().info(f"Path found between {coord1} and {coord2}: {path}")
        return path

    
