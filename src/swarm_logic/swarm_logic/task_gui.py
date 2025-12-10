
"""
Task GUI Node - A standalone ROS2 node with a Tkinter GUI for publishing tasks.

This node:
- Subscribes to 'shelf_info' topic to get items and their drop-off locations
- Subscribes to 'send_map' topic to validate pickup locations (status 0 = valid)
- Allows users to input item_id and pickup location
- Publishes TaskInfo with pickup and drop-off locations

Run with: ros2 run swarm_logic task_gui_node
"""

import rclpy
from rclpy.node import Node
from messages.msg import TaskInfo, Map
import threading
import tkinter as tk
from tkinter import ttk, messagebox


class TaskGuiNode(Node):
    """ROS2 Node that publishes TaskInfo messages and tracks map/shelf data."""
    
    def __init__(self):
        super().__init__('task_gui_node')
        
        # Publisher for new tasks
        self.task_pub = self.create_publisher(TaskInfo, 'new_tasks', 10)
        self.task_id_counter = 1
        
        # Items dictionary: item_id -> (drop_x, drop_y)
        self.items = {}
        
        # Map dictionary: (x, y) -> status (0 = valid/free, else blocked)
        self.map_dict = {}
        
        # Subscribe to shelf info for items and drop-off locations
        self.shelf_sub = self.create_subscription(
            Map, 'shelf_info', self.shelf_callback, 4000
        )
        
        # Subscribe to map data for location validation
        self.map_sub = self.create_subscription(
            Map, 'send_map', self.map_callback, 10
        )
        
        self.get_logger().info("Task GUI Node initialized")
        self.get_logger().info("Subscribing to 'shelf_info' for items/drop-offs")
        self.get_logger().info("Subscribing to 'send_map' for location validation")
        self.get_logger().info("Publishing to 'new_tasks'")
    
    def shelf_callback(self, msg: Map):
        """Handle shelf info - creates item with drop-off location."""
        item_id = msg.status  # status field holds the item_id
        drop_x = msg.x
        drop_y = msg.y
        
        if item_id not in self.items:
            self.items[item_id] = (drop_x, drop_y)
            self.get_logger().info(f"Item {item_id} registered | Drop-off: ({drop_x}, {drop_y})")
    
    def map_callback(self, msg: Map):
        """Handle map data - updates location validity."""
        x = msg.x
        y = msg.y
        status = msg.status
        
        # Update map (status 0 = valid/free)
        self.map_dict[(x, y)] = status
        self.get_logger().debug(f"Map update: ({x}, {y}) = {status}")
    
    def is_location_valid(self, x: int, y: int) -> bool:
        """Check if a pickup location is valid (status 0 = valid)."""
        # If we have no map data yet, allow all locations
        if not self.map_dict:
            return True
        
        # Check if location exists in map and has status 0 (valid)
        key = (int(x), int(y))
        if key in self.map_dict:
            return self.map_dict[key] == 0
        
        # Location not in map - assume valid (could be unexplored)
        return True
    
    def get_drop_location(self, item_id: int) -> tuple:
        """Get drop-off location for an item. Returns None if not found."""
        return self.items.get(item_id)
    
    def get_available_items(self) -> list:
        """Get list of available item IDs."""
        return list(self.items.keys())
    
    def publish_task(self, pickup_x: float, pickup_y: float, 
                     dropoff_x: float, dropoff_y: float, item_id: int, item_qty: int = 1) -> int:
        """Publish a task with pickup and drop-off locations."""
        msg = TaskInfo()
        msg.id = self.task_id_counter
        msg.item_id = item_id
        msg.item_qty = item_qty
        
        # Set pickup location
        msg.x = float(pickup_x)
        msg.y = float(pickup_y)
        
        # Set drop-off location
        msg.x_drop = float(dropoff_x)
        msg.y_drop = float(dropoff_y)
        
        self.task_pub.publish(msg)
        
        task_id = self.task_id_counter
        self.get_logger().info(
            f"Published Task {task_id} | Item: {item_id} x{item_qty} | "
            f"Pickup: ({pickup_x:.1f}, {pickup_y:.1f}) -> Drop: ({dropoff_x:.1f}, {dropoff_y:.1f})"
        )
        
        self.task_id_counter += 1
        return task_id


class TaskGuiApp:
    """Tkinter GUI Application for task publishing."""
    
    def __init__(self, ros_node: TaskGuiNode):
        self.ros_node = ros_node
        self.map_width = 60
        self.map_height = 60
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Task Publisher GUI")
        self.root.geometry("420x450")
        self.root.resizable(False, False)
        
        # Set up the GUI
        self._setup_gui()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _setup_gui(self):
        """Set up all GUI elements."""
        # Main frame with padding
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Title
        title_label = ttk.Label(
            main_frame, 
            text="Task Publisher", 
            font=("Helvetica", 16, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Subtitle
        subtitle_label = ttk.Label(
            main_frame, 
            text="Enter pickup location and select item",
            font=("Helvetica", 9)
        )
        subtitle_label.grid(row=1, column=0, columnspan=2, pady=(0, 15))
        
        # Current Task ID display
        self.task_id_var = tk.StringVar(value=f"Next Task ID: {self.ros_node.task_id_counter}")
        task_id_label = ttk.Label(main_frame, textvariable=self.task_id_var, font=("Helvetica", 10))
        task_id_label.grid(row=2, column=0, columnspan=2, pady=(0, 5))
        
        # Available items display (with word wrap for long lists)
        self.items_var = tk.StringVar(value="Available Items: (waiting for shelf data...)")
        items_label = ttk.Label(main_frame, textvariable=self.items_var, font=("Helvetica", 9), wraplength=350)
        items_label.grid(row=3, column=0, columnspan=2, pady=(0, 15))
        
        # Item ID
        item_label = ttk.Label(main_frame, text="Item ID:", font=("Helvetica", 11))
        item_label.grid(row=4, column=0, sticky="e", padx=(0, 10), pady=5)
        
        self.item_entry = ttk.Entry(main_frame, width=15, font=("Helvetica", 11))
        self.item_entry.grid(row=4, column=1, sticky="w", pady=5)
        self.item_entry.insert(0, "1")
        
        # Item Quantity
        qty_label = ttk.Label(main_frame, text="Quantity:", font=("Helvetica", 11))
        qty_label.grid(row=5, column=0, sticky="e", padx=(0, 10), pady=5)
        
        self.qty_entry = ttk.Entry(main_frame, width=15, font=("Helvetica", 11))
        self.qty_entry.grid(row=5, column=1, sticky="w", pady=5)
        self.qty_entry.insert(0, "1")
        
        # Pickup X Coordinate
        x_label = ttk.Label(main_frame, text="Pickup X:", font=("Helvetica", 11))
        x_label.grid(row=6, column=0, sticky="e", padx=(0, 10), pady=5)
        
        self.x_entry = ttk.Entry(main_frame, width=15, font=("Helvetica", 11))
        self.x_entry.grid(row=6, column=1, sticky="w", pady=5)
        self.x_entry.insert(0, "30")
        
        # Pickup Y Coordinate
        y_label = ttk.Label(main_frame, text="Pickup Y:", font=("Helvetica", 11))
        y_label.grid(row=7, column=0, sticky="e", padx=(0, 10), pady=5)
        
        self.y_entry = ttk.Entry(main_frame, width=15, font=("Helvetica", 11))
        self.y_entry.grid(row=7, column=1, sticky="w", pady=5)
        self.y_entry.insert(0, "30")
        
        # Drop-off display (read-only, determined by item)
        drop_label = ttk.Label(main_frame, text="Drop-off:", font=("Helvetica", 11))
        drop_label.grid(row=8, column=0, sticky="e", padx=(0, 10), pady=5)
        
        self.drop_var = tk.StringVar(value="(select item)")
        drop_display = ttk.Label(main_frame, textvariable=self.drop_var, font=("Helvetica", 11))
        drop_display.grid(row=8, column=1, sticky="w", pady=5)
        
        # Update drop-off when item entry changes
        self.item_entry.bind('<KeyRelease>', self._update_dropoff_display)
        
        # Publish Button
        self.publish_btn = ttk.Button(
            main_frame, 
            text="Publish Task", 
            command=self._publish_task
        )
        self.publish_btn.grid(row=9, column=0, columnspan=2, pady=20)
        
        # Status display
        self.status_var = tk.StringVar(value="Ready to publish tasks")
        self.status_label = ttk.Label(
            main_frame, 
            textvariable=self.status_var, 
            font=("Helvetica", 10),
            foreground="green"
        )
        self.status_label.grid(row=10, column=0, columnspan=2, pady=(10, 0))
        
        # Bind Enter key to publish
        self.root.bind('<Return>', lambda e: self._publish_task())
        
        # Start periodic update for available items display
        self._update_items_display()
    
    def _update_items_display(self):
        """Periodically update the available items display."""
        items = self.ros_node.get_available_items()
        if items:
            self.items_var.set(f"Available Items: {sorted(items)}")
        else:
            self.items_var.set("Available Items: (waiting for shelf data...)")
        
        # Schedule next update
        self.root.after(1000, self._update_items_display)
    
    def _update_dropoff_display(self, event=None):
        """Update the drop-off location display based on selected item."""
        try:
            item_id = int(self.item_entry.get())
            drop_loc = self.ros_node.get_drop_location(item_id)
            if drop_loc:
                self.drop_var.set(f"({drop_loc[0]}, {drop_loc[1]})")
            else:
                self.drop_var.set("(item not found)")
        except ValueError:
            self.drop_var.set("(invalid item)")
    
    def _validate_input(self) -> tuple:
        """Validate and parse input values."""
        x_str = self.x_entry.get()
        y_str = self.y_entry.get()
        item_str = self.item_entry.get()
        
        # Parse item ID
        try:
            item_id = int(item_str)
        except ValueError:
            raise ValueError("Item ID must be a valid integer")
        
        # Check if item exists
        drop_loc = self.ros_node.get_drop_location(item_id)
        if drop_loc is None:
            available = self.ros_node.get_available_items()
            if available:
                raise ValueError(f"Item {item_id} not found. Available: {sorted(available)}")
            else:
                raise ValueError("No items available yet. Waiting for shelf data...")
        
        # Parse coordinates
        try:
            x = int(float(x_str))
            y = int(float(y_str))
        except ValueError:
            raise ValueError("X and Y must be valid numbers")
        
        # Check bounds
        if not (0 <= x <= self.map_width):
            raise ValueError(f"X must be between 0 and {self.map_width}")
        if not (0 <= y <= self.map_height):
            raise ValueError(f"Y must be between 0 and {self.map_height}")
        
        # Validate pickup location against map
        if not self.ros_node.is_location_valid(x, y):
            raise ValueError(f"Pickup location ({x}, {y}) is blocked! Choose another location.")
        
        # Parse quantity
        try:
            qty = int(self.qty_entry.get())
            if qty < 1:
                raise ValueError("Quantity must be at least 1")
        except ValueError:
            raise ValueError("Quantity must be a valid positive integer")
        
        return x, y, item_id, drop_loc, qty
    
    def _publish_task(self):
        """Handle publish button click."""
        try:
            pickup_x, pickup_y, item_id, drop_loc, qty = self._validate_input()
            dropoff_x, dropoff_y = drop_loc
            
            # Publish the task
            task_id = self.ros_node.publish_task(
                pickup_x, pickup_y, dropoff_x, dropoff_y, item_id, qty
            )
            
            # Update status
            self.status_var.set(
                f"Task {task_id}: Item {item_id} x{qty} | ({pickup_x},{pickup_y})->({dropoff_x},{dropoff_y})"
            )
            self.status_label.config(foreground="green")
            
            # Update next task ID display
            self.task_id_var.set(f"Next Task ID: {self.ros_node.task_id_counter}")
            
        except ValueError as e:
            self.status_var.set(f"Error: {e}")
            self.status_label.config(foreground="red")
    
    def _on_closing(self):
        """Handle window close event."""
        self.root.destroy()
    
    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


# Commented out separate classes as requested - functionality incorporated into TaskGuiNode
# class items_alloc(Node):
#     #this will get the shelf data and then use that to define the number of items and their drop offs
#     def __init__(self):
#         super().__init__("item_alloc")
#         self.shelf_sub=self.create_subscription(Map,"shelf_info",self.create_items,10)
#         self.bot_sub=self.create_subscription(Map,"bot_info",self.bot_callback,10)
#         
#         self.items={}    #will have items and their drop offs
#
#     def create_items(self,msg:Map):
#         x=msg.x
#         y=msg.y
#         id=msg.status
#         self.items[id]=(x,y)   #so drop off and the corresponding item id is defined like this
#
# class MapProxy(Node):
#     #this would collect the data from that published by the bots, as a dictionary
#     #msg Map, topic send_map
#     def __init__(self):
#         super().__init__("MapProxy_Node")
#         self.map_sub=self.create_subscription(Map,"send_map",self.map_callback,10)
#         self.map_dict={}
#
#     def map_callback(self,msg:Map):
#         self.get_logger().info(f"received info about ({msg.x},{msg.y}) as {msg.status}")
#         x=msg.x
#         y=msg.y
#         status=msg.status
#         if (x,y) not in self.map_dict:
#             self.map_dict[(x,y)]=status


def main(args=None):
    """Main entry point for the task GUI node."""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create the ROS2 node
    ros_node = TaskGuiNode()
    
    # Create a thread for ROS2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    try:
        # Create and run the GUI (must be in main thread)
        app = TaskGuiApp(ros_node)
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
