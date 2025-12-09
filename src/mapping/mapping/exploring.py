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
    def loadmap(self):
        plt.clf()
        plt.imshow(self.data, vmin = 0, vmax = 3)                     
        plt.colorbar()                       
        plt.draw() 
        plt.pause(0.000001)
    def update_data(self):
        for (x,y), value in self.map.grid.items():
            self.data[x][y] = value
    def see(self):
        for b in self.bots:
            b.see()
    def distance(self, coord1: tuple, coord2: tuple):
        return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])
    def sort_frontiers(self,i):
        for f in self.map.Frontier.keys():
            self.frontiercosts[i][f] = self.distance(self.bots[i].coord,f)
    def cost(self, coord1: tuple, coord2: tuple):
        direct_distance = len(self.pathplanner.nav(coord1, coord2))
        return direct_distance
    def old_cost(self, coord1: tuple, coord2: tuple,x:int):
        direct_distance = len(self.pathplanner.van(coord1, coord2,x))
        return direct_distance
    def oldmethod(self,i):
        self.sort_frontiers(i)
        min_cost = 10**18
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
        self.get_logger().info(f"Bot {i} choosing frontier")
        self.get_logger().info(f"Current frontiers: {self.map.Frontier.keys()}")
        def return_coord(best_coord):
            if best_coord == (-1,-1):
                return None
            self.pastchosen_Frontiers.add(best_coord)
            del self.frontiercosts[i][best_coord]
            self.get_logger().info(f"Bot {i} chose frontier {best_coord}")
            del self.map.Frontier[best_coord]
            return best_coord
        inf = 10**18
        mincost = (-1,-1,inf)
        to_remove = []
        x = len(self.frontiercosts[i])
        for coord in self.frontiercosts[i].keys():
            if coord not in self.map.Frontier.keys():
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
                         # if new frontiers were found
            best_coord = (mincost[0],mincost[1])
            break 
            
             
        return return_coord(best_coord)



def main():
    rclpy.init(args=None)
    swarm = Swarm(1)
    swarm.see()
    swarm.map.update_frontiers(0)

    # Compute initial paths for each bot
    paths = []
    for i in range(swarm.bot_count):
        path = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))
        paths.append(path)

    # Step loop: move each bot one step per iteration
    while len(swarm.map.Frontier.keys()) > 0:
        for i in range(swarm.bot_count):
            if paths[i]:  # if bot still has steps
                next_step = paths[i].pop(0)  # take first step
                swarm.bots[i].move(next_step)
            else:
                paths[i] = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))
                if paths[i]:  # if a new path was found
                    next_step = paths[i].pop(0) 
                    swarm.bots[i].move(next_step)
        swarm.loadmap()
    plt.draw()
    

if __name__ == '__main__':
    main()
        
