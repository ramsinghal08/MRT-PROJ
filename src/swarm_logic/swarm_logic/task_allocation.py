import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time
from messages.msg import TaskInfo
from messages.msg import Map
from messages.msg import TaskInfoPath   #this will be used to send the info about the path to the path planning team
from messages.msg import BotMove   # For publishing bot movements during region segregation
from .branch import branch, get_branch_node

from .bot import bot  


class Task:
    def __init__(self, id, x, y, drop_x, drop_y, item_qty=1):
        self.id = id
        self.x = x
        self.y = y
        self.drop_x = drop_x
        self.drop_y = drop_y
        self.item_qty = item_qty  # Number of items in this task


class RobotProxy:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.task_queue = []
        self.max_capacity = 3

    def is_overloaded(self):
        return (len(self.task_queue) >= self.max_capacity)


class BotProxy:
    
    def __init__(self, id, priority, coord, color="white", returning=False):
        self.id = id
        self.priority = priority
        self.coord = coord
        self.color = color
        self.returning = returning
        self.branch_id = None
        self.follow_leader = None
        
        # Task assignment tracking
        self.current_items = 0      # 0-3 items carrying (0 = free, >0 = busy)
        self.current_task = None    # Current task object
        self.is_leader = False      # Leader flag
        self.members = []           # List of member BotProxy (only for leaders)
        self.region_id = None       # Region this bot belongs to
    
    def is_free(self):
       
        return self.current_items == 0
    
    def assign_items(self, count, task):
        
        self.current_items = min(3, count)
        self.current_task = task
    
    def complete_task(self):
        
        self.current_items = 0
        self.current_task = None
    
    def set_leader(self):
        self.color = "blue"
        self.is_leader = True
    
    def set_follower(self):
        self.color = "green"
        self.is_leader = False
    
    def set_returning(self):
        self.returning = True
        self.color = "red"
    
    def move(self, coord: tuple):
        self.coord = coord



class TaskAllocator(Node):
    def __init__(self, map_width=60, map_height=60, region_size=20):
        super().__init__('task_allocation_node')
        self.map_width = map_width
        self.map_height = map_height
        self.region_size = region_size
        self.cols = math.ceil(map_width / region_size)

        self.region_leaders = {}
        self.all_leaders = []
        self.task_publishers = {}
        
        # Bot and item tracking (combined from items_and_bot_alloc)
        self.bot_dict = {}       # bot_id  (x, y) initial coords
        self.items = {}          # item_id  (drop_x, drop_y)
        self.bot_objects = {}    # bot_id BotProxy object
        self.active_branches = {}  # branch_id branch object
        self.current_tasks = {}  # bot_id task_id (bots currently assigned to tasks)
        self.pending_movements = {}  # bot_id target (x, y) for initial movement tracking
        self.branches_ready = False  # Flag: True after all bots have successfully moved to regions
        
        # Map data for location validation (combined from MapProxy)
        self.map_dict = {}       # (x, y)  status (0 = free, else blocked)
        
        # Subscriptions
        self.create_subscription(TaskInfo, 'new_tasks', self.task_callback, 10)
        self.create_subscription(Map, 'shelf_info', self.shelf_callback, 10)
        self.create_subscription(Map, 'bot_info', self.bot_callback, 10)
        # Enabled with larger queue to store map data
        self.create_subscription(Map, 'send_map', self.map_callback, 4000)
        self.create_subscription(Map, 'bot_task_info', self.bot_task_complete_callback, 40)
        # self.task_pub_to_path=self.create_publisher(TaskInfoPath, 'task_info_path', 10)

        self.task_pub_to_path=self.create_publisher(BotMove, 'bot_move', 10)
       
        self.bot_move_pub = self.create_publisher(BotMove, 'bot_move', 10)
        
        self.get_logger().info("Task allocator initialised. Waiting for bot registration...")

   
    
    def shelf_callback(self, msg: Map):
        """Handle shelf info - creates item with drop-off location."""
        x = msg.x
        y = msg.y
        item_id = msg.status
        if item_id not in self.items:
            self.items[item_id] = (x, y)
            self.get_logger().info(f"Item {item_id} registered | Drop-off: ({x}, {y})")

    
    
    def bot_callback(self, msg: Map):
        
        x = msg.x
        y = msg.y
        bot_id = msg.status  # Using status field for bot_id
        
       
        if x == 0 and y == 0 and bot_id == 0:
            # All bots have been sent - trigger branch creation
            self.get_logger().info("All bots have been registered!")
            self.get_logger().info(f"Total bots: {len(self.bot_dict)}")
            self.create_branches_from_bots()
        else:
            # Only register if not already registered (avoid duplicates from multiple calls)
            if bot_id not in self.bot_dict:
                self.bot_dict[bot_id] = (x, y)
                self.get_logger().info(f"Bot {bot_id} registered at ({x}, {y}) [NEW]")
            else:
                # Update position if already exists
                self.bot_dict[bot_id] = (x, y)
                self.get_logger().debug(f"Bot {bot_id} position updated to ({x}, {y})")

   
    
    def map_callback(self, msg: Map):
        """Handle map data - updates location validity."""
        x = msg.x
        y = msg.y
        status = msg.status
        self.map_dict[(x, y)] = status
        self.get_logger().debug(f"Map update: ({x}, {y}) = {status}")

    def bot_task_complete_callback(self, msg: Map):
        
        bot_id = msg.status
        new_x = msg.x
        new_y = msg.y
        
        # Initial movement tracking (during branch setup)
        if bot_id in self.pending_movements:
            target = self.pending_movements[bot_id]
            target_x, target_y = target
            
            # Check if position is close to target (allow small deviation)  .... because initially sometimes it deviates from target maybe due to phase 1 commands overlapping
            distance = abs(new_x - target_x) + abs(new_y - target_y)
            if distance > 5:  # If too far from target, this is likely an old message
                self.get_logger().debug(f"Bot {bot_id} at ({new_x}, {new_y}) - ignoring, too far from  {target}")
                return
            
            # Bot reached target (or close enough)
            self.pending_movements.pop(bot_id)
            self.get_logger().info(f"Bot {bot_id} reached position ({new_x}, {new_y}) [Target: {target}]")
            
            # Update bot position
            if bot_id in self.bot_objects:
                self.bot_objects[bot_id].coord = (new_x, new_y)
            self.bot_dict[bot_id] = (new_x, new_y)
            
            # Check if all bots have completed initial movement
            if len(self.pending_movements) == 0 and not self.branches_ready:
                self.branches_ready = True
                self.get_logger().info("=== ALL BOTS REACHED THEIR POSITIONS - READY FOR TASKS ===")
            else:
                self.get_logger().info(f"Waiting for {len(self.pending_movements)} more bots: {list(self.pending_movements.keys())}")
            return
        
        #  Task completion tracking
        if bot_id in self.current_tasks:
            self.get_logger().info(f"Bot {bot_id} task completed | New position: ({new_x}, {new_y})")
            
            # Remove from current tasks tracking
            task_id = self.current_tasks.pop(bot_id)
            self.get_logger().info(f"Bot {bot_id} removed from task {task_id}")
            
            # Get the bot object and reset it
            bot = self.bot_objects.get(bot_id)
            if bot is None:
                self.get_logger().warn(f"Bot {bot_id} not found in bot_objects!")
                return
            
            # Reset bot capacity (make it free for new tasks)
            bot.complete_task()
            
            # Update bot's position
            bot.coord = (new_x, new_y)
            self.bot_dict[bot_id] = (new_x, new_y)
            
            # Find nearest leader and reassign bot to that leader's team
            if not bot.is_leader:  # Only reassign members, not leaders
                nearest_leader = self.find_nearest_leader(new_x, new_y)
                old_leader = self._get_current_leader_of_bot(bot)
                if nearest_leader and nearest_leader != old_leader:
                    if old_leader and bot in old_leader.members:
                        old_leader.members.remove(bot)
                        self.get_logger().info(f"Bot {bot_id} removed from Leader {old_leader.id}'s team")
                    
                    nearest_leader.members.append(bot)
                    self.get_logger().info(f"Bot {bot_id} reassigned to Leader {nearest_leader.id}'s team")
            
            self.get_logger().info(f"Bot {bot_id} is now FREE and ready for new tasks")
            return
        
        # Just a position update (not initial movement or task completion)
        self.get_logger().debug(f"Bot {bot_id} position update at ({new_x}, {new_y})")
        if bot_id in self.bot_objects:
            self.bot_objects[bot_id].coord = (new_x, new_y)
            self.bot_dict[bot_id] = (new_x, new_y)

    def _get_current_leader_of_bot(self, bot):
        
        for leader in self.all_leaders:
            if bot in leader.members:
                return leader
        return None


    def is_location_free(self, x: int, y: int) -> bool:
       
        key = (int(x), int(y))
        if key in self.map_dict:
            return self.map_dict[key] == 0
        # If not in map, assume free (unexplored)
        return True

    def find_free_location_near(self, target_x: int, target_y: int) -> tuple:
   
        if self.is_location_free(target_x, target_y):
            return (target_x, target_y)
        
        # Search in expanding squares around target
        for radius in range(1, 10):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nx, ny = target_x + dx, target_y + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if self.is_location_free(nx, ny):
                            return (nx, ny)
        
        # Fallback to target even if blocked
        return (target_x, target_y)

    def _publish_bot_move(self, bot_id: int, init_pos: tuple, final_pos: tuple):
        
        msg = BotMove()
        msg.bot_id = bot_id
        msg.init_x = int(init_pos[0])
        msg.init_y = int(init_pos[1])
        msg.final_x = int(final_pos[0])
        msg.final_y = int(final_pos[1])
        self.bot_move_pub.publish(msg)
        self.get_logger().info(f"Published movement for Bot {bot_id}: ({init_pos}) -> ({final_pos})")
        time.sleep(1.0) 
    
    def create_branches_from_bots(self):
        
        num_bots = len(self.bot_dict)
        if num_bots < 2:
            self.get_logger().warn("Not enough bots to create branches (need at least 2)")
            return
        
        num_branches = num_bots // 2
        self.get_logger().info(f"Creating {num_branches} branches from {num_bots} bots")
        
        # Create BotProxy objects for all bots
        bot_ids = sorted(self.bot_dict.keys())
        priority = 1
        for bot_id in bot_ids:
            coord = self.bot_dict[bot_id]
            self.bot_objects[bot_id] = BotProxy(
                id=bot_id,
                priority=priority,
                coord=coord
            )
            priority += 1
        
        # Calculate region centers based on number of branches
        region_centers = self._calculate_region_centers(num_branches)
        
        # Divide bots into branches
        bots_per_branch = num_bots // num_branches
        remaining_bots = num_bots % num_branches
        
        bot_list = [self.bot_objects[bid] for bid in bot_ids]
        bot_index = 0
        
        self.get_logger().info(f"Bot IDs to assign: {bot_ids}")
        self.get_logger().info(f"Bots per branch: {bots_per_branch}, Remaining: {remaining_bots}")
        
        for branch_idx in range(num_branches):
            # Determine number of bots for this branch
            branch_size = bots_per_branch
            if branch_idx < remaining_bots:
                branch_size += 1
            
            # First bot becomes leader, rest are members
            leader = bot_list[bot_index]
            members = bot_list[bot_index + 1 : bot_index + branch_size]
            bot_index += branch_size
            
            self.get_logger().info(f"Branch {branch_idx}: Leader={leader.id}, Members={[m.id for m in members]}, Size={branch_size}")
            
            # Get region center for this branch
            region_center = region_centers[branch_idx]
            
            # Find free location near region center for leader
            leader_coord = self.find_free_location_near(region_center[0], region_center[1])
            
            # Store initial position and move leader to region center
            leader_init = self.bot_dict[leader.id]
            leader.coord = leader_coord
            leader.set_leader()  # Mark as leader
            leader.region_id = branch_idx  # Track region
            leader.members = members  # Store members list in leader
            self.get_logger().info(f"Branch {branch_idx + 1}: Leader Bot {leader.id} moved to {leader_coord}")
            
            # Publish leader movement and track it
            self._publish_bot_move(leader.id, leader_init, leader_coord)
            self.pending_movements[leader.id] = leader_coord  # Track pending movement
            time.sleep(1.0)
            # Move members near leader (not too close)
            for i, member in enumerate(members):
                member.set_follower()  # Mark as follower
                member.region_id = branch_idx  # Track region
                
                # Offset members - spread them out in the region (not close to leader)
                offset_x = ((i % 3) - 1) * 3  # -3, 0, 3
                offset_y = ((i // 3) - 1) * 3
                member_target = (leader_coord[0] + offset_x, leader_coord[1] + offset_y)
                member_coord = self.find_free_location_near(member_target[0], member_target[1])
                member_init = self.bot_dict[member.id]
                member.coord = member_coord
                self.get_logger().info(f"  Member Bot {member.id} moved to {member_coord}")
                
                # Publish member movement and track it
                self._publish_bot_move(member.id, member_init, member_coord)
                self.pending_movements[member.id] = member_coord  # Track pending movement
                time.sleep(1.0)
            # Create the branch using the branch class from branch.py
            try:
                new_branch = branch(
                    members=members,
                    leader=leader,
                    path=[leader_coord]
                )
                self.active_branches[new_branch.id] = new_branch
                self.get_logger().info(f"Branch {new_branch.id} created successfully")
            except Exception as e:
                self.get_logger().error(f"Failed to create branch: {e}")
                # Fallback: store branch info without using branch class
                self.active_branches[branch_idx + 1] = type('SimpleBranch', (), {
                    'id': branch_idx + 1,
                    'leader': leader,
                    'members': members,
                    'path': [leader_coord]
                })()
        
        self.get_logger().info(f"Created {len(self.active_branches)} branches")
        
        # Wait for bot movements to complete on the map
        self.get_logger().info("Waiting for bot movements to complete...")
        time.sleep(3.0)
        self.get_logger().info("Bot movements complete. Ready for task allocation.")
        
        # Register leaders with task allocation
        self.register_leaders_from_exploration(self.active_branches)

    def _calculate_region_centers(self, num_branches: int) -> list:
       
        centers = []
        
        if num_branches == 1:
            centers.append((self.map_width // 2, self.map_height // 2))
        elif num_branches == 2:
            # Divide horizontally into 2 regions
            centers.append((self.map_width // 4, self.map_height // 2))
            centers.append((3 * self.map_width // 4, self.map_height // 2))
        elif num_branches <= 4:
            # Divide into 4 quadrants
            quadrant_centers = [
                (self.map_width // 4, self.map_height // 4),
                (3 * self.map_width // 4, self.map_height // 4),
                (self.map_width // 4, 3 * self.map_height // 4),
                (3 * self.map_width // 4, 3 * self.map_height // 4),
            ]
            centers = quadrant_centers[:num_branches]
        else:
            # Grid-based division for more branches
            cols = math.ceil(math.sqrt(num_branches))
            rows = math.ceil(num_branches / cols)
            cell_width = self.map_width // cols
            cell_height = self.map_height // rows
            
            for i in range(num_branches):
                col = i % cols
                row = i // cols
                cx = col * cell_width + cell_width // 2
                cy = row * cell_height + cell_height // 2
                centers.append((cx, cy))
        
        return centers
    
    def register_leaders_from_exploration(self, active_branches):
        
        self.get_logger().info("Registering leaders from branches...")

        for branch_id, branch_obj in active_branches.items():
            leader_bot = branch_obj.leader  # This is already a BotProxy
            
            region_id = self.get_region_id(leader_bot.coord[0], leader_bot.coord[1])
            leader_bot.region_id = region_id

            # Store leader directly (BotProxy already has members list)
            self.region_leaders[region_id] = leader_bot
            self.all_leaders.append(leader_bot)

            topic_name = f'/bot_{leader_bot.id}/assign_task'
            pub = self.create_publisher(TaskInfo, topic_name, 10)
            self.task_publishers[leader_bot.id] = pub

            self.get_logger().info(f"Leader {leader_bot.id} assigned to Region {region_id} | Members: {[m.id for m in leader_bot.members]}")
        
        self.get_logger().info(f"Registration complete. {len(self.all_leaders)} leaders ready.")

    
    def get_region_id(self, x, y):
        col = int(x // self.region_size)
        row = int(y // self.region_size)
        return row * self.cols + col

    def find_nearest_leader(self, x, y):
        
        if not self.all_leaders:
            return None
        
        best_leader = None
        min_dist = float('inf')
        
        for leader in self.all_leaders:
            # Calculate distance from pickup to leader's position
            dist = math.sqrt((leader.coord[0] - x)**2 + (leader.coord[1] - y)**2)
            if dist < min_dist:
                min_dist = dist
                best_leader = leader
        
        return best_leader

    def find_free_member(self, leader):
        
        if leader is None:
            return None
        
        for member in leader.members:
            if member.is_free():
                return member
        return None

    def find_other_leader_with_free_member(self, exclude_leader):
        
        for leader in self.all_leaders:
            if leader != exclude_leader:
                if self.find_free_member(leader) is not None:
                    return leader
        return None

    def assign_task(self, task):
      
        remaining_items = task.item_qty
        
        # Find nearest leader to pickup location
        current_leader = self.find_nearest_leader(task.x, task.y)
        
        if current_leader is None:
            self.get_logger().warn(f"Task {task.id}: No leaders registered!")
            return False
        
        self.get_logger().info(f"Task {task.id}: Pickup at ({task.x}, {task.y}) -> Nearest Leader {current_leader.id} | Members: {[m.id for m in current_leader.members]}")
        
        bots_assigned = 0
        
        while remaining_items > 0:
            # Find a free member
            member = self.find_free_member(current_leader)
            
            if member is None:
                # Try other leader's members
                other_leader = self.find_other_leader_with_free_member(current_leader)
                if other_leader is None:
                    self.get_logger().warn(f"Task {task.id}: No free bots available! {remaining_items} items unassigned.")
                    break
                current_leader = other_leader
                member = self.find_free_member(current_leader)
                if member is None:
                    self.get_logger().warn(f"Task {task.id}: No free bots available! {remaining_items} items unassigned.")
                    break
            
            # Assign up to 3 items to this member
            items_to_assign = min(3, remaining_items)
            member.assign_items(items_to_assign, task)
            remaining_items -= items_to_assign
            bots_assigned += 1
            
            # Track this task assignment
            self.current_tasks[member.id] = task.id
            
            self.get_logger().info(f"Task {task.id}: Assigned {items_to_assign} items to Bot {member.id}")
            
            # Send command to path planning
            self.send_command_to_bot(member.id, task, items_to_assign)
        
        return bots_assigned > 0

    def find_free_neighbor_cell(self, x: int, y: int) -> tuple:
        
        x, y = int(x), int(y)
        neighbors = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]  # Up, Down, Left, Right
        
        for nx, ny in neighbors:
            key = (nx, ny)
            if key in self.map_dict and self.map_dict[key] == 0:
                return (nx, ny)
        
        # If no free neighbor found in map_dict, return the first valid coordinate
        # (in case map_dict is incomplete)
        for nx, ny in neighbors:
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                return (nx, ny)
        
        # Fallback to original
        return (x, y)

  
    def send_command_to_bot(self, bot_id, task, items_assigned=1):
         """Send task info to path planning."""
         msg = BotMove()
         msg.bot_id = bot_id
         # msg.task_id = task.id
         # msg.pick_x = float(task.x)
         # msg.pick_y = float(task.y)
         # msg.place_x = float(task.drop_x)
         # msg.place_y = float(task.drop_y)
         msg.type="update"
         msg.init_x=int(task.x)
         msg.init_y=int(task.y)
         # msg.final_x=int(task.drop_x)
         # msg.final_y=int(task.drop_y)
         drop_x, drop_y = self.find_free_neighbor_cell(int(task.drop_x), int(task.drop_y))
         msg.final_x=int(drop_x)
         msg.final_y=int(drop_y)
         # Get bot's current position
         bot = self.bot_objects.get(bot_id)
         # if bot:
         #     msg.bot_x = int(bot.coord[0])
         #     msg.bot_y = int(bot.coord[1])
         # else:
         #     msg.bot_x = int(self.bot_dict.get(bot_id, (0, 0))[0])
         #     msg.bot_y = int(self.bot_dict.get(bot_id, (0, 0))[1])
       
         self.task_pub_to_path.publish(msg)
         self.get_logger().info(f"Published TaskInfoPath for Bot {bot_id} | Task {task.id} | Items: {items_assigned}")

    def task_callback(self, msg):
        """Handle incoming task from GUI."""
        self.get_logger().info(f"Received task {msg.id} | Item qty: {msg.item_qty} | Current leaders: {len(self.all_leaders)}")
        new_task = Task(msg.id, msg.x, msg.y, msg.x_drop, msg.y_drop, msg.item_qty)
        self.assign_task(new_task)




def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
