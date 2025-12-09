#here we will make the basic class of the bots from which their attributes will me controlled

import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap
from messages.srv import Mapinfo
from messages.msg import Map
class bot(Node):
    def __init__(self,id,parent, map ,coord=(1,1),color="white",returning=False,):
        super().__init__("bot_node")
        parent.map.grid[coord]= 2
        self.id=id
        self.coord=coord
        self.color=color
        self.returning=returning
        self.path=[]
        self.follow_leader=None    #initially following none
        self.LOS=3  #line of sight
        self.get_map = self.create_client(Mapinfo, 'map_info')
        self.map = map
        self.send_map = self.create_publisher(Map,'send_map',10)
        self.parent = parent #the swarm object that created this bot
        self.i = 0
    def get_map_info(self,x,y):
        while not self.get_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = Mapinfo.Request()
        request.x = x
        request.y = y
        future = self.get_map.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status 
    def sendmap(self,x,y,status):
        msg = Map()
        msg.x = x 
        msg.y = y
        msg.status = status
        self.send_map.publish(msg)

    def see(self):
        cx, cy = self.coord 
        blocked_set = set()

        x_range = sorted(range(cx - self.LOS + 1, cx + self.LOS), key=lambda k: abs(k - cx))
        y_range = sorted(range(cy - self.LOS + 1, cy + self.LOS), key=lambda k: abs(k - cy))

        def is_opaque(p):
            return self.map.grid.get(p) == 1 or p in blocked_set

        for x in x_range:
            for y in y_range:
                if (x == cx and y == cy):
                    continue
                dx = x - cx
                dy = y - cy 
                is_blocked = False
                if abs(dx) > 0 and abs(dy) > 0:
                    sign_x = 1 if dx > 0 else -1
                    sign_y = 1 if dy > 0 else -1
                    point_1 = (x - sign_x, y)
                    point_2 = (x, y - sign_y)
                    
                    if is_opaque(point_1) and is_opaque(point_2):
                        is_blocked = True
                else:
                    prev_x = x - (1 if dx > 0 else -1) if dx != 0 else x
                    prev_y = y - (1 if dy > 0 else -1) if dy != 0 else y
                    
                    if (prev_x, prev_y) != (cx, cy) and is_opaque((prev_x, prev_y)):
                        is_blocked = True
    
                if is_blocked:
                    blocked_set.add((x, y))
                else:
                    if (x, y) in self.map.grid:
                        val = self.get_map_info(x, y)
                        self.map.grid[(x, y)] = val
                        self.sendmap(x, y, val)

        is_blocked = False
        for dx in range(1, self.LOS):
            if self.map.grid.get((cx + dx, cy)) == 1:
                is_blocked = True
                break
        if not is_blocked:
            target = (cx + self.LOS, cy)
            if target in self.map.grid:
                val = self.get_map_info(target[0], target[1])
                self.map.grid[target] = val
                self.sendmap(target[0], target[1], val)

        is_blocked = False
        for dy in range(1, self.LOS):
            if self.map.grid.get((cx, cy + dy)) == 1:
                is_blocked = True
                break
        if not is_blocked:
            target = (cx, cy + self.LOS)
            if target in self.map.grid:
                val = self.get_map_info(target[0], target[1])
                self.map.grid[target] = val
                self.sendmap(target[0], target[1], val)
    
        is_blocked = False
        for dx in range(1, self.LOS):
            if self.map.grid.get((cx - dx, cy)) == 1:
                is_blocked = True
                break
        if not is_blocked:
            target = (cx - self.LOS, cy)
            if target in self.map.grid:
                val = self.get_map_info(target[0], target[1])
                self.map.grid[target] = val
                self.sendmap(target[0], target[1], val)
        
        is_blocked = False
        for dy in range(1, self.LOS):
            if self.map.grid.get((cx, cy - dy)) == 1:
                is_blocked = True
                break
        if not is_blocked:
            target = (cx, cy - self.LOS)
            if target in self.map.grid:
                val = self.get_map_info(target[0], target[1])
                self.map.grid[target] = val
                self.sendmap(target[0], target[1], val)
    
        self.parent.map.grid[self.coord] = 2
        self.parent.update_data()
        self.parent.map.update_frontiers(self.id)
    
    def move(self,coord: tuple):
        self.i += 1
        if coord == (self.coord[0]+1,self.coord[1]) or (self.coord[0]-1,self.coord[1]) or (self.coord[0],self.coord[1]+1) or (self.coord[0],self.coord[1]-1) or (coord==self.coord):
            self.coord = coord
            self.see()
        for (x,y) in self.parent.frontiercosts[self.id].keys():
                self.parent.frontiercosts[self.id][(x,y)] -= 1

                 
        self.parent.loadmap()
    def follow_path(self,path):
        for i in range(len(path)):
            self.move(path[i])
    def leader(self):
        self.color="blue"
        self.update_map()
    def follower(self):
        self.color="green"
        self.update_map()
    def explorer(self):
        self.color="yellow"
        self.update_map()
    def is_returning(self):
        self.returning=True
        self.color="red"
        self.update_map()
    def follow(self,leader):
        self.follow_leader=leader
    def is_leader(self):
        return self.color=="blue"
    def update_map(self): #called everytime a bot object is made or colour updated so that map can be updated
        #publish the id and colour of bot 
        pass

