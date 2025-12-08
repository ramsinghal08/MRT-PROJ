
import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from mapping.mapping_shi import GeneratedMap
from messages.srv import Mapinfo
from messages.msg import Map
from messages.msg import Roverpath
from path_planning.path import pathplanning
#import Explored map 

class publishpath(Node): 
    def __init__(self): #map is a class with map.grid being a dictionary with key (x,y) and value status, with 0 for clear 
        super().__init__("path_planning_node")
        self.map =  #Load the explored map here
        self.pathplanner = pathplanning(self.map)
        self.get_logger().info("Path Planning Node has been started")
        self.publisher=self.create_publisher(Roverpath,'path', 10)


    def publish_path(self,request,response):
        response= self.pathplanner(map,request[0],request[1])
        self.publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)

    node = publishpath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 