import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16

class GeneratedMap: #generated  by bots, shared by all of them
    def __init__(self,parent,map_size = 60):
        self.parent = parent
        self.grid = {}
        self.Frontier = {}
        self.shelves = {}
        for x in range(0,map_size):
            for y in range(0,map_size):
                self.grid[(x,y)] = 3 #unexplored




    def get(self,x, y):
        if (x, y) in self.grid:
            return self.grid[(x, y)]
        else:
            return 3  #out of bounds 

    def setValue(self,x, y, value): #0 for free, 1 for obtacled, No value for unexplored
        self.grid[(x, y)] = value

    def remove(self,x, y):
        if (x, y) in self.grid:
            del self.grid[(x, y)]

    def search(self,value):
        result = []
        for coord, v in self.grid.items():
            if v == value:
                result.append(coord)
        return result


    def neighbors(self,x, y):
        # 4-directional neighbors
        dirs = [(1,0), (-1,0), (0,1), (0,-1)]
        return [(x+dx, y+dy) for dx, dy in dirs]


    def bounds(self):
        # square map bounds
        if not self.grid:
            return None

        xs = [x for (x, y) in self.grid.keys()]
        ys = [y for (x, y) in self.grid.keys()]
        return min(xs), max(xs), min(ys), max(ys)


    def update_frontiers(self,id):
        self.Frontier = {}
        for (x, y) in self.grid:
          if self.grid[(x,y)] == 0 and (x,y):
            for nx, ny in self.neighbors(x, y):
                if self.grid[(nx, ny)] == 3:  # unexplored neighbor
                    self.Frontier[(x, y)] = id # marking which bot found this frontier
                    break

    def identify_shelves(self):
        shelves = {}
        visited = set()
        current_id = 1
        known_coords = list(self.grid.keys())

        for (x, y) in known_coords:
            val = self.get(x, y)
            if val == 1 and (x, y) not in visited and 0<x<60 and 0<y<60 :
                group = []
                queue = [(x, y)]
                visited.add((x, y))
                while queue:
                    cx, cy = queue.pop(0)
                    group.append((cx, cy))
                    for nx, ny in self.neighbors(cx, cy):
                        if self.get(nx, ny) == 1 and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            queue.append((nx, ny))

                xs = [p[0] for p in group]
                ys = [p[1] for p in group]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                
                if (min_x == 0 or max_x == 59 or min_y == 0 or max_y == 59):
                    continue
                shelf_x = max_x
                shelf_y = max_y
                for i in range(min_x, max_x + 1):
                    for j in range(min_y, max_y + 1):
                        self.setValue(i, j, 4)
                        visited.add((i, j))

                shelves[current_id] = (shelf_x, shelf_y)
                current_id += 1
        self.shelves = shelves
    
    def printmap(self):
        # render the known explored region __ is so python treats it as a special function and prints a string when we code print(str)
        b = self.bounds()
        if b is None:
            print("<empty map>")

        minx, maxx, miny, maxy = b
        rows = []
        for y in range(miny, maxy + 1):
            row = []
            for x in range(minx, maxx + 1):
                row.append(str(self.get(x, y)))
            rows.append(" ".join(row))
        print("\n".join(rows))


