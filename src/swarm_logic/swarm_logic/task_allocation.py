import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
from messages.msg import TaskInfo

class Task:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        
class RobotProxy:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.task_queue = []
        self.max_capacity = 3

    def is_overloaded(self):
        return (len(self.task_queue) >= self.max_capacity)

class TaskAllocator(Node):
    def __init__(self, map_width=60, map_height=60, region_size=20):
        super().__init__('task_allocation_node')
        self.map_width=map_width
        self.map_height=map_height
        self.region_size=region_size
        self.cols = math.ceil(map_width/region_size)

        self.region_leaders={}
        self.all_leaders=[]
        self.task_publishers={}
        self.create_subscription(TaskInfo, 'new_tasks', self.task_callback, 10)   #taskinfo
        self.get_logger().info("Task allocator initialised. Waiting for Phase 2")

    def register_leaders_from_exploration(self, active_branches):
        self.get_logger().info("Phase 1 Complete. Registering leaders")

        for branch_id, branch_obj in active_branches.items():
            leader_bot = branch_obj.leader

            proxy= RobotProxy(leader_bot.id, leader_bot.coord[0], leader_bot.coord[1])
            region_id = self.get_region_id(proxy.x, proxy.y)

            self.region_leaders[region_id]=proxy
            self.all_leaders.append(proxy)

            topic_name = f'/bot_{proxy.id}/assign_task'
            pub = self.create_publisher(TaskInfo, topic_name, 10)
            self.task_publishers[proxy.id] = pub

            self.get_logger().info(f"Leader {proxy.id} assigned to Region {region_id}")

    def get_region_id(self, x, y):
        col= int(x//self.region_size)
        row= int(y//self.region_size)
        return row*self.cols + col

    def assign_best_robot(self,task):
        region_id = self.get_region_id(task.x, task.y)
        local_leader = self.region_leaders.get(region_id)

        target_robot = None

        if local_leader and not local_leader.is_overloaded():
            target_robot = local_leader
            self.get_logger().info(f"Task {task.id}: Assigned locally to bot {target_robot.id}")

        else:
            self.get_logger().info(f"Task {task.id}: Local leader busy/missing. Searching neighbors")
            target_robot= self.find_nearest_available_neighbor(task.x, task.y)

        if target_robot:
            target_robot.task_queue.append(task)
            self.send_command_to_bot(target_robot.id, task)
            return True
        else:
            self.get_logger().warn(f"Task {task.id}: CRITIICAL = All bots overloaded.")
            return False 
    
    def find_nearest_available_neighbor(self, tx, ty):
        best_bot = None
        min_dist = float('inf')

        for bot in self.all_leaders:
            if not bot.is_overloaded():
                dist = math.sqrt((bot.x - tx)**2 + (bot.y- ty)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_bot = bot 
        return best_bot

    def send_command_to_bot(self, bot_id, task):
        if bot_id in self.task_publishers:
            msg= TaskInfo()
            msg.id = task.id
            msg.x = float(task.x)
            msg.y = float(task.y)
            self.task_publishers[bot_id].publish(msg)

    def task_callback(self, msg):
        new_task = Task(msg.id, msg.x, msg.y)
        self.assign_best_robot(new_task)

def main(args=None):
    rclpy.init(args=args)
    node= TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()