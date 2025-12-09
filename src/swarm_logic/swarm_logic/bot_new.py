import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from messages.msg import PathMsg
from messages.msg import VisualMsg
from messages.srv import MoveMembers
from messages.srv import LeaderMove
import time

# Global universal node instance
_bot_node_instance = None


def get_bot_node():
    
    global _bot_node_instance
    if _bot_node_instance is None:
        if not rclpy.ok():
            raise RuntimeError("rclpy not initialized. Call rclpy.init() first.")
        _bot_node_instance = BotNode()
    return _bot_node_instance


class Bot:
    
    def __init__(self, id, priority, coord=(0, 0), color="white", returning=False, map=None):
        self.id = id
        self.coord = coord
        self.color = color
        self.priority = priority
        self.returning = returning
        self.path = []
        self.follow_leader = None  # initially following none
        self.LOS = 3  # line of sight
        self.map = map
        self.branch_id = None  # Will be set externally if this bot is a leader
        
        # Automatically register this bot with the universal node
        node = get_bot_node()
        node.add_bot(self)
        
        # Publish new registration to visual node
        node.publish_visual_update(self, "new_registration")

    def update_coord(self, coord: tuple):
        """Update bot's coordinate - pure Python logic"""
        self.coord = coord

    def is_valid_move(self, coord: tuple) -> bool:
        """Check if the move is valid (adjacent cell)"""
        valid = [
            (self.coord[0] + 1, self.coord[1]),
            (self.coord[0] - 1, self.coord[1]),
            (self.coord[0], self.coord[1] + 1),
            (self.coord[0], self.coord[1] - 1),
        ]
        # return coord in valid  #no need for random movement for demo
        return True

    def move(self, coord: tuple):
       
        node = get_bot_node()
        node.move_bot(self, coord)

    def set_leader(self):
      
        self.color = "blue"
        # Publish color change to visual node
        node = get_bot_node()
        node.publish_visual_update(self, "color_change")
        time.sleep(1.0)
    def set_follower(self):
      
        self.color = "green"
        # Publish color change to visual node
        node = get_bot_node()
        node.publish_visual_update(self, "color_change")
        time.sleep(1.0)

    def set_explorer(self):
        
        self.color = "yellow"
        # Publish color change to visual node
        node = get_bot_node()
        node.publish_visual_update(self, "color_change")
        time.sleep(1.0)

    def set_is_returning(self):
        
        self.returning = True
        self.color = "red"
        # Publish color change to visual node
        node = get_bot_node()
        node.publish_visual_update(self, "color_change")
        time.sleep(1.0)

    def follow(self, leader):
        
        self.follow_leader = leader

    def is_leader(self) -> bool:
       
        return self.color == "blue"

    def get_info(self) -> dict:
        
        return {
            'id': self.id,
            'coord': self.coord,
            'color': self.color,
            'priority': self.priority,
            'returning': self.returning,
            'is_leader': self.is_leader()
        }


class BotNode(Node):
  
    def __init__(self):
        super().__init__('bot_node_ros')
        
        # Dictionary to store all bot objects with their ID as key
        self.bots = {}
        
       
        self.create_service(LeaderMove, 'leader_info', self.move_callback)
        
        
        self.move_member_client = self.create_client(MoveMembers, 'move_members')
        
        
        self.send_coord_to_branch = self.create_publisher(PathMsg, 'path_topic', 10)
        
        # Publisher for visual updates  (for demo)
        self.visual_pub = self.create_publisher(VisualMsg, 'visual_updates', 10)
        
        self.get_logger().info("BotNode initialized")

    def add_bot(self, bot):
        
        self.bots[bot.id] = bot
        self.get_logger().info(f"Bot {bot.id} added to node. Total bots: {len(self.bots)}")

    def remove_bot(self, bot_id: int):
        
        if bot_id in self.bots:
            del self.bots[bot_id]
            self.get_logger().info(f"Bot {bot_id} removed from node")

    def get_bot(self, bot_id: int):
        
        self.get_logger().info("giving info about a bot")
        return self.bots.get(bot_id, None)

    def move_callback(self, request, response):
        
        self.get_logger().info(f"Received move request for Bot {request.id} to {request.coord}")
        
        coord = tuple(request.coord)
        bot_id = request.id
        
        # Get the bot object from dictionary
        bot_obj = self.bots.get(bot_id)
        
        if bot_obj is None:
            self.get_logger().error(f"Bot {bot_id} not found in bots dictionary")
            response.success = False
            return response
        
        # Call the move method
        self.move_bot(bot_obj, (coord[0], coord[1]))
        response.success = True
        
        return response

    def move_bot(self, bot, coord: tuple):
       
        # Check if move is valid using bot's logic
        # if not bot.is_valid_move(coord):
        #     self.get_logger().warn(f"Invalid move for Bot {bot.id} to {coord}")
        #     return  # invalid move, ignore

        # Update bot's coordinate (pure Python operation)
        bot.update_coord(coord)

        # If not leader → done
        if not bot.is_leader():
            self.get_logger().info(f"Bot {bot.id} moved to {coord} as follower.")
            self.publish_update_map(bot)
            time.sleep(1.0)
            return
        else:
            self.get_logger().info(f"Bot {bot.id} moved to {coord} as leader.")
            self.publish_update_map(bot)
            time.sleep(1.0)
        # ----- LEADER logic (ROS2 operations) -----
        # Leader must notify branch to move members
        if bot.branch_id is None:
            self.get_logger().warn(f"Leader Bot {bot.id} has no branch_id set")
            self.publish_update_map(bot)
            time.sleep(1.0)
            return

        while not self.move_member_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for move_members service...')

        req = MoveMembers.Request()
        req.branch_id = bot.branch_id
        req.x, req.y = coord

        future = self.move_member_client.call_async(req)
        # # WAIT UNTIL MEMBERS FINISH MOVING
        # rclpy.spin_until_future_complete(self, future)

        # # Check result
        # if future.result() and future.result().success:

        # WAIT UNTIL MEMBERS FINISH MOVING
        # Use a timeout to prevent hanging indefinitely
        # Poll with small sleeps to give the executor time to process callbacks
        timeout_sec = 30.0  # Maximum time to wait for members
        start_time = time.time()
        
        while not future.done():
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self.get_logger().error(f"Timeout waiting for members of Branch {bot.branch_id} to move!")
                break
            time.sleep(0.1)  # Small sleep to yield to executor
        
        if future.done():
            result = future.result()
            if result and result.success:
                self.get_logger().info(f"Branch {bot.branch_id} followers moved to {coord}")
            else:
                self.get_logger().warn(f"Branch {bot.branch_id} members failed to move (success=False).")
        else:
            self.get_logger().warn(f"Branch {bot.branch_id} members move did not complete in time.")
        
        self.publish_update_map(bot)
        time.sleep(1.0)

    def follow_path(self, bot_id: int, path: list):
      
        bot = self.get_bot(bot_id)
        if bot is None:
            self.get_logger().error(f"Bot {bot_id} not found")
            return
        
        for coord in path:
            self.move_bot(bot, coord)

    def set_bot_as_leader(self, bot_id: int):
      
        bot = self.get_bot(bot_id)
        if bot:
            bot.set_leader()
            self.publish_update_map(bot)
            self.get_logger().info(f"Bot {bot_id} set as leader")

    def set_bot_as_follower(self, bot_id: int):
        
        bot = self.get_bot(bot_id)
        if bot:
            bot.set_follower()
            self.publish_update_map(bot)
            self.get_logger().info(f"Bot {bot_id} set as follower")

    def set_bot_as_explorer(self, bot_id: int):
      
        bot = self.get_bot(bot_id)
        if bot:
            bot.set_explorer()
            self.publish_update_map(bot)
            self.get_logger().info(f"Bot {bot_id} set as explorer")

    def set_bot_returning(self, bot_id: int):
       
        bot = self.get_bot(bot_id)
        if bot:
            bot.set_returning()
            self.publish_update_map(bot)
            self.get_logger().info(f"Bot {bot_id} set as returning")

    def publish_update_map(self, bot):
        
        # Publish position update to visual node
        self.publish_visual_update(bot, "update_position")
    
    def publish_visual_update(self, bot, update_type):
       
        msg = VisualMsg()
        msg.type = update_type
        msg.bot_id = bot.id
        msg.color = bot.color
        msg.coord_x = int(bot.coord[0])
        msg.coord_y = int(bot.coord[1])
        
        self.visual_pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info(f"Published visual update: {update_type} for Bot {bot.id}")

    def publish_path_to_branch(self, bot_id: int, path: list):
       
        bot = self.get_bot(bot_id)
        if bot is None:
            return
        
        msg = PathMsg()
        
        self.send_coord_to_branch.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    bot_node = get_bot_node()
    rclpy.spin(bot_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()