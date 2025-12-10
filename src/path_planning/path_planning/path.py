import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap
import heapq

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
        self.grid = getattr(map, 'grid', map)
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
        if start == end:
            return [start]
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""
        # Create start and end node
        start_node = Point(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Point(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = set()
        g_score = {start_node.position:0}
        # Add the start node
        open_list.append((start_node.f,0,start_node))

        # Add a maximum iteration limit to prevent infinite loops
        iterations = 0

        # Loop until you find the end
        while len(open_list) > 0:
            #if iterations > max_iterations:
             #   self.get_logger().info("Pathfinding exceeded maximum iterations. Terminating.")
              #  return []

            # Get the current node
            f,k,current_node = heapq.heappop(open_list)

            # Pop current off open list, add to closed list
            closed_list.add(current_node.position)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]  # Return reversed path

            # Generate children
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure walkable terrain
                if self.grid.get(node_position, 1) != 0:
                    continue

                # Create new node
                child = Point(current_node, node_position)
                # Child is on the closed list
                if child.position in closed_list:
                    continue
                # Create the f, g, and h values
                dx = abs(child.position[0] - end_node.position[0])
                dy = abs(child.position[1] - end_node.position[1])
                child.g = current_node.g + 1
                child.h = max(dx,dy)+ (min(dx,dy))*0.4142  # Using Octile distance for heuristic
                child.f = child.g + child.h

                # Child is already in the open list
                if child.position in g_score and child.g >= g_score[child.position]: 
                    continue
                # Add the child to the open list
                heapq.heappush(open_list, (child.f,iterations,child))
                g_score[child.position] = child.g
                iterations += 1
        return []  # No path found
        
        
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


