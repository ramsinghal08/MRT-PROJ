import rclpy 
import sys
from rclpy.node import Node
from std_msgs.msg import Int16
from messages.srv import Mapinfo
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

class mapgeneration(Node): # 1 = filled 0 = blank 
    def __init__(self):
        super().__init__('map_generator')
        self.map_size = 100
        self.resolution = 0.5
        self.Map = self.makemap()
        self.map_publisher_ = self.create_service(Mapinfo, 'map_info',self.handle_map_request)
    def makemap(self,size=60, num_attempts=2000, min_gap=2):
        grid = np.zeros((size, size), int)

        grid[0, :] = grid[-1, :] = 1
        grid[:, 0] = grid[:, -1] = 1

        for _ in range(num_attempts):

            w = np.random.randint(2, 5)
            h = np.random.randint(4, 12)

            if np.random.random() < 0.5:
                w, h = h, w

            x = np.random.randint(1, size - w - 1)
            y = np.random.randint(1, size - h - 1)

            check_x_start = max(0, x - min_gap)
            check_x_end   = min(size, x + w + min_gap)
            check_y_start = max(0, y - min_gap)
            check_y_end   = min(size, y + h + min_gap)

            sub_grid = grid[check_x_start:check_x_end, check_y_start:check_y_end]


            if np.sum(sub_grid) == 0:
                grid[x : x + w, y : y + h] = 1
        return grid
    def show_map(self):
            plt.imshow(self.Map)                     
            plt.colorbar()                       
            plt.show()    




    def handle_map_request(self, request, response):
        response.status = int(self.Map[request.x][request.y])
        return response 
def main(args=None):
    rclpy.init(args=args)
    map_generator = mapgeneration()
    rclpy.spin(map_generator)
    map_generator.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main() 

