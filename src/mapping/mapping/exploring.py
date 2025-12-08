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
    pastchosen_Frontiers = set()
    def __init__(self, bot_count=1):
        super().__init__('swarm')
        self.map = GeneratedMap()
        self.bots = []
        self.bot_count = bot_count
        self.pathplanner = pathplanning(self.map)
        self.data = np.full((60,60),3)  # Initialize a 60x60 map with unknown values
        for i in range(bot_count):
            new_bot = bot(id=i,parent = self,map=self.map)
            self.bots.append(new_bot)
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
    
    def cost(self, coord1: tuple, coord2: tuple,x:int): #I used coord1 as bots current location in the later code, pls confirm

        #coord1, coord2, and frontier points are tuples (x, y)


        direct_distance = len(self.pathplanner.van(coord1, coord2,x))
        """""
        approached_count = 0
        increased_count = 0

        for fp in self.map.Frontier:
            dist1 = len(self.pathplanner.nav(fp, coord1))
            dist2 = len(self.pathplanner.nav(fp, coord2))

            if dist2 < dist1:
                approached_count += 1
            elif dist2 > dist1:
                increased_count += 1

        DISTANCE_WEIGHT = 1.0
        APPROACHED_WEIGHT = 2.0
        INCREASED_WEIGHT = 1.5

        # avoid division by zero
        if increased_count == 0:
            increased_count = 1 
        

        total_cost = (direct_distance * (approached_count ** APPROACHED_WEIGHT)/(increased_count ** INCREASED_WEIGHT))
        """

        return direct_distance
    def chooseFrontier(self,i): #if leader
        self.get_logger().info(str(self.map.Frontier))
        self.get_logger().info("chosing frontier")
        self.get_logger().info(str(len(self.map.Frontier)))
        costs = {}
        cost = (0,0,10**18)
        for (x,y) in self.map.Frontier:
          if (x,y) not in Swarm.pastchosen_Frontiers: 
            costs[(x,y)]= self.cost(self.bots[i].coord, (x,y),cost[2])
            if costs[(x,y)]:
                cost = (x,y,costs[(x,y)])
        best_coord = (cost[0],cost[1])
        self.get_logger().info(f"Best frontier chosen at {best_coord} with cost {cost[2]}")
        Swarm.pastchosen_Frontiers.add(best_coord)
        return best_coord



def main():
    rclpy.init(args=None)
    swarm = Swarm(3)
    swarm.see()
    swarm.map.update_frontiers()

    # Compute initial paths for each bot
    paths = []
    for i in range(swarm.bot_count):
        path = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))
        paths.append(path)

    # Step loop: move each bot one step per iteration
    while len(swarm.map.Frontier) > 0:
        all_done = True
        for i in range(swarm.bot_count):
            if paths[i]:  # if bot still has steps
                next_step = paths[i].pop(0)  # take first step
                swarm.bots[i].move(next_step)
                all_done = False

        if all_done:
            # recompute paths once frontiers change
            for i in range(swarm.bot_count):
                paths[i] = swarm.pathplanner.nav(swarm.bots[i].coord, swarm.chooseFrontier(i))

if __name__ == '__main__':
    main()
        
