import rclpy 
import sys
from rclpy.node import Node
from path_planning.path import pathplanning
from messages.msg import Roverpath
from messages.msg import Map

class publishpath(Node):
    def __init__(self):
        self.map={}
        super().__init__("publish_path_node")
        #edit topic name later
        self.task_subscription_ = self.create_subscription(Roverpath, "topic_name", self.task_callback, 10)
        self.map_subscription_ = self.create_subscription(Map, "send_map", self.map_callback, 10)
        self.path_publisher_ = self.create_publisher(Roverpath, 'path_info', 10)

        self.timer = self.create_timer(1.0, self.path_publisher_)

    def task_callback(self, msg):
        self.start=msg.path[0]
        self.end=msg.path[1]
        self.id=msg.roverid

    def map_callback(self, msg):
        new_point={(msg.x,msg.y):msg.status}
        self.map.update(new_point)        


    def publish_path(self):
        msg=Roverpath
        self.pathplanner = pathplanning(self.map)
        path = self.pathplanner.nav(self.start, self.end)
        msg.roverid=self.id
        msg.path=path
        self.path_publisher_.publish(msg)

def main():
    rclpy.init(args=None)
    node = publishpath()  
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
