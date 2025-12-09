
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
            

    def nav(self,start:tuple,end:tuple): ## elementary path finding algorithim for now please keep the input and output types same. 
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""
        # Create start and end node
        start_node = Point(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Point(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                #if node_position[0] > (60) or node_position[0] < 0 or node_position[1] > (60) or node_position[1] < 0:
                #    continue

                # Make sure walkable terrain
                if self.grid.get(node_position,1) != 0:
                    continue

                # Create new node
                new_node = Point(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                skip=False 
                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        skip=True
                        

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:  
                    if child == open_node and child.g > open_node.g :
                        skip=True
                    if skip==True:
                        continue 
                # Add the child to the open list
                open_list.append(child)
        
    def van(self,coord1:tuple,coord2:tuple,i:int):
        if coord2 == None:
            return []
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
        return path
