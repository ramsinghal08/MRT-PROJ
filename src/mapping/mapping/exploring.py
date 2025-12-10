import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap
from messages.srv import Mapinfo
from messages.msg import Map 
from swarm_logic.bot import bot 
import matplotlib.pyplot as plt      # Imports matplotlib for plotting
import numpy as np                   # Imports numpy to create/load numeric data
from path_planning.path import pathplanning
from messages.msg import BotMove 
                      
class Swarm(Node):

    def __init__(self, bot_count=1):
        super().__init__('swarm')
        self.map = GeneratedMap(self)
        self.bots = []
        self.bot_count = bot_count
        self.pathplanner = pathplanning(self.map)
        self.data = np.full((60,60),3)  # Initialize a 60x60 map with unknown values
        for i in range(bot_count):
            new_bot = bot(id=i,parent = self,map=self.map)
            self.bots.append(new_bot)
        self.pastchosen_Frontiers = set()
        self.frontiercosts = [{} for _ in range(bot_count)]
        self.sendbotinfo = self.create_publisher(Map,'bot_info',10)
        self.sendshelves = self.create_publisher(Map,'shelf_info',10)
        self.to_move = self.create_subscription(BotMove,'bot_move',self.updatetasks,10)
        self.sentmap = []
    def loadmap(self):
        plt.clf()
        plt.imshow(self.data, vmin = 0, vmax = 4)
        for label, coord in self.map.shelves.items():
            plt.text(coord[1], coord[0], str(label), color='red', fontsize=8)               
        plt.colorbar()                       
        plt.draw() 
        plt.pause(0.000001)
    def updatetasks(self,msg):
        self.get_logger().info(f"got movement request for {msg.bot_id}:{msg.type}")
        if msg.type=="update":
            i = msg.bot_id
            x2 = msg.final_x
            y2 = msg.final_y
            x1 = msg.init_x
            y1 = msg.init_y
            self.bots[i].path1 = self.pathplanner.nav(self.bots[i].coord,(x1,y1))
            self.bots[i].path2 = self.pathplanner.nav((x1,y1),(x2,y2))
            self.get_logger().info(f"bot{i} going to {(x1,y1)} and then {(x2,y2)}")
        else:
            i = msg.bot_id
            x2 = msg.final_x
            y2 = msg.final_y
            self.bots[i].path2 = self.pathplanner.nav(self.bots[i].coord,(x2,y2))
            self.bots[i].path1 = []
            self.get_logger().info(f"bot{i} going to {(x2,y2)}")
            
    def send_bot_info(self):
        for i in range(self.bot_count):
            msg = Map()
            msg.x = self.bots[i].coord[0]
            msg.y = self.bots[i].coord[1]
            msg.status = self.bots[i].id
            self.sendbotinfo.publish(msg)
        msg=Map()
        msg.x=0
        msg.y=0
        msg.status=0
        self.sendbotinfo.publish(msg)
        self.get_logger().info("the bots data is sent ")
    def send_shelves(self):
        for label, coord in self.map.shelves.items():
            msg = Map()
            msg.x = coord[0]
            msg.y = coord[1]
            msg.status = label
            self.sendshelves.publish(msg)
    def update_data(self):
        for (x,y), value in self.map.grid.items():
            self.data[x][y] = value
    def see(self):
        for b in self.bots:
            b.see()
    def distance(self, coord1: tuple, coord2: tuple):
        return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])
    def sort_frontiers(self,i):
        self.frontiercosts[i] = {}
        for f in self.map.Frontier.keys():
            if f not in self.pastchosen_Frontiers:
                self.frontiercosts[i][f] = self.distance(self.bots[i].coord,f)
    def cost(self, coord1: tuple, coord2: tuple):
        direct_distance = len(self.pathplanner.nav(coord1, coord2))
        return direct_distance
    def old_cost(self, coord1: tuple, coord2: tuple,x:int):
        direct_distance = len(self.pathplanner.van(coord1, coord2,x))
        return direct_distance
    def oldmethod(self,i):
        if not self.map.Frontier:
            return self.bots[i].coord
        self.sort_frontiers(i)
        if not self.frontiercosts[i]:
            return self.bots[i].coord
        coord, min_v = min(self.frontiercosts[i].items(), key=lambda x: x[1])
        while True:
            self.frontiercosts[i][coord] = self.cost(self.bots[i].coord,coord)
            coordx, min_v = min(self.frontiercosts[i].items(), key=lambda x: x[1])
            if coordx == coord:
                break
            else:
                coord = coordx
        return coord

    
    def chooseFrontier(self,i): #if leader
        if not self.map.Frontier:
            return self.bots[i].coord
        def return_coord(best_coord):
            if best_coord == (-1,-1):
                return self.bots[i].coord
            if best_coord == self.bots[i].coord:
                return best_coord
            self.pastchosen_Frontiers.add(best_coord)
            del self.frontiercosts[i][best_coord]
            return best_coord
        inf = 10**18
        mincost = (-1,-1,inf)
        to_remove = []
        for coord in self.frontiercosts[i].keys():
            if (coord not in self.map.Frontier.keys()) or (coord in self.pastchosen_Frontiers):
                to_remove.append(coord)
        for coord in to_remove:
            del self.frontiercosts[i][coord]
        for f in self.map.Frontier.keys():
            if (f not in self.frontiercosts[i].keys()) and self.map.Frontier[f] == i:
                self.frontiercosts[i][f] = self.cost(self.bots[i].coord,f)
                if self.frontiercosts[i][f] < mincost[2]:
                    mincost = (f[0],f[1],self.frontiercosts[i][f])   
        
        if (mincost[0],mincost[1]) == (-1,-1): #no new frontiers uncovered
            best_coord = self.oldmethod(i)
            return return_coord(best_coord)
        while True:
            coord, min_v = min(self.frontiercosts[i].items(), key=lambda x: x[1])
            if min_v<mincost[2]: # checking if a previous found frontier is better
                self.frontiercosts[i][coord] = self.cost(self.bots[i].coord,coord)
                if self.frontiercosts[i][coord] < mincost[2]:
                    best_coord = coord
                    break
            #if new frontiers is better
            best_coord = (mincost[0],mincost[1])
            break 
            
             
        return return_coord(best_coord)



def main():
    rclpy.init(args=None)
    swarm = Swarm(10)
    swarm.see()
    swarm.map.update_frontiers(0)

    # Compute initial paths for each bot
    paths = []
    for i in range(swarm.bot_count):
        path = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))
        next_step = path.pop(0) if path else swarm.bots[i].coord
        swarm.bots[i].move(next_step)
        paths.append(path)

    # Step loop: move each bot one step per iteration
    while len(swarm.map.Frontier.keys()) > 0:
        for i in range(swarm.bot_count):
            if paths[i]:  # if bot still has steps
                next_step = paths[i].pop(0)  # take first step
                swarm.bots[i].move(next_step)
            else:
                if len(swarm.map.Frontier.keys())>0:
                    paths[i] = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))
                if paths[i]:  # if a new path was found
                    next_step = paths[i].pop(0) 
                    swarm.bots[i].move(next_step)
        for x in range(swarm.bot_count):
            swarm.map.grid[swarm.bots[x].coord] = 2
        swarm.update_data()
        swarm.loadmap()
    for i in range(swarm.bot_count):
        if paths[i]:
            while paths[i]:
                next_step = paths[i].pop(0)
                swarm.bots[i].move(next_step)
                for x in range(swarm.bot_count):
                    swarm.map.grid[swarm.bots[x].coord] = 2
                swarm.update_data()
                swarm.loadmap()

    swarm.map.identify_shelves()
    swarm.update_data()
    swarm.get_logger().info(f"Shelves: {swarm.map.shelves}")
    swarm.loadmap()
    swarm.send_bot_info()
    swarm.send_shelves()
    while True:
        rclpy.spin_once(swarm,timeout_sec = 0.1)
        for i in range(swarm.bot_count):
            if swarm.bots[i].path1:
                next_step = swarm.bots[i].path1.pop(0)
                swarm.bots[i].justmove(next_step)
            elif swarm.bots[i].path2:
                next_step = swarm.bots[i].path2.pop(0)
                swarm.bots[i].justmove(next_step)
                if not swarm.bots[i].path2:
                    msg = Map()
                    msg.status = swarm.bots[1].id
                    msg.x = swarm.bots[i].coord[0]
                    msg.y = swarm.bots[i].coord[1]
                    swarm.sendbotinfo.publish(msg)
        for x in range(swarm.bot_count):
            swarm.map.grid[swarm.bots[x].coord] = 2
        swarm.update_data()
        swarm.loadmap()
    

if __name__ == '__main__':
    main()
        
