#here we will make the basic class of the bots from which their attributes will me controlled

import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap
from messages.srv import Mapinfo
from messages.msg import Map
class bot(Node):
    def __init__(self,id,parent = None,coord=(1,1),color="white",returning=False,map=GeneratedMap()):
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
        points = []
        for x in range(1,self.LOS+1):
            v = self.get_map_info(self.coord[0]+x,self.coord[1])
            self.map.setValue(self.coord[0]+x,self.coord[1],v)
            self.sendmap(x,self.coord[1],v)
            points.append((self.coord[0]+x,self.coord[1]))
            if v == 1:
                break
        for x in range(1,self.LOS+1):
            v = self.get_map_info(self.coord[0]-x,self.coord[1])
            self.map.setValue(self.coord[0]-x,self.coord[1],v)
            self.sendmap(x,self.coord[1],v)
            points.append((self.coord[0]-x,self.coord[1]))
            if v == 1:
                break

        for y in range(1,self.LOS+1):
            v = self.get_map_info(self.coord[0],self.coord[1]+y)
            self.map.setValue(self.coord[0],self.coord[1]+y,v)
            self.sendmap(self.coord[0],y,v)
            points.append((self.coord[0],self.coord[1]+y))
            if v == 1:
                break
        for y in range(1,self.LOS+1):
            v = self.get_map_info(self.coord[0],self.coord[1]-y)
            self.map.setValue(self.coord[0],self.coord[1]-y,v)
            self.sendmap(self.coord[0],y,v)
            points.append((self.coord[0],self.coord[1]-y))
            if v == 1:
                break
        self.parent.map.grid[self.coord]= 2
        self.parent.update_data()
        self.parent.map.update_frontiers()
    def move(self,coord: tuple):
        if coord == (self.coord[0]+1,self.coord[1]) or (self.coord[0]-1,self.coord[1]) or (self.coord[0],self.coord[1]+1) or (self.coord[0],self.coord[1]-1):
            self.coord = coord
            self.see()
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

