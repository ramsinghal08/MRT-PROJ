#only for swarm demo simulation...

import rclpy
from rclpy.node import Node
from messages.msg import VisualMsg
from messages.msg import MapDataMsg
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import threading


class VisualNode(Node):
    """
    ROS2 Node for real-time visualization of swarm bot exploration.
    Displays the map grid and bot positions using matplotlib.
    """
    
    def __init__(self):
        super().__init__('visual_node')
        self.get_logger().info("Visual node initializing...")
        
        # Subscribe to visual updates (bot registration, color changes, position updates)
        self.visual_sub = self.create_subscription(
            VisualMsg,
            'visual_updates',
            self.visual_callback,
            10
        )
        
        # Subscribe to map data
        self.map_sub = self.create_subscription(
            MapDataMsg,
            'map_data',
            self.map_callback,
            10
        )
        
        # Map data storage
        self.map_width = None
        self.map_height = None
        self.map_grid = None
        self.map_initialized = False
        
        # Track visited cells: set of (x, y) tuples
        self.visited_cells = set()
        
        # Bot tracking: {bot_id: {'coord': (x, y), 'color': 'colorname', 'circle': Circle, 'label': Text}}
        self.bots = {}
        
        # Color mapping for map cells
        self.cell_colors = {
            1: '#A0A0A0',  # Light grey for traversable
            2: '#8B4513',  # Brown for 2-way split
            3: '#FF69B4',  # Hot pink for 3-way split
        }
        
        # Color for visited/traversed cells
        self.visited_color = '#E6F3FF'  # Light blue for visited cells
        
        # Matplotlib setup
        self.fig = None
        self.ax = None
        self.plot_lock = threading.Lock()
        
        # Store cell rectangles for updating visited cells
        self.cell_patches = {}
        
        # Initialize plot
        self._init_plot()
        
        self.get_logger().info("Visual node initialized. Waiting for map data...")
    
    def _init_plot(self):
        """Initialize the matplotlib figure and axes."""
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.ax.set_title('Swarm Bot Exploration Map', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Coordinate', fontsize=12)
        self.ax.set_ylabel('Y Coordinate', fontsize=12)
        self.ax.set_aspect('equal')
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def map_callback(self, msg):
        """Handle incoming map data message."""
        self.get_logger().info(f"Received map data: {msg.width}x{msg.height}")
        
        with self.plot_lock:
            self.map_width = msg.width
            self.map_height = msg.height
            
            # Reconstruct grid from flattened data (row-major order)
            self.map_grid = np.array(msg.grid_data).reshape((msg.height, msg.width))
            
            self._draw_map()
            self.map_initialized = True
            
        self.get_logger().info("Map visualization complete.")
    
    def _draw_map(self):
        """Draw the map grid with colored cells."""
        self.ax.clear()
        self.ax.set_title('Swarm Bot Exploration Map', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Coordinate', fontsize=12)
        self.ax.set_ylabel('Y Coordinate', fontsize=12)
        
        # Clear cell patches storage
        self.cell_patches = {}
        
        # Draw each cell as a colored rectangle
        for y in range(self.map_height):
            for x in range(self.map_width):
                cell_value = int(self.map_grid[y, x])
                
                # Check if cell has been visited
                if (x, y) in self.visited_cells:
                    color = self.visited_color
                else:
                    color = self.cell_colors.get(cell_value, '#FFFFFF')
                
                rect = patches.Rectangle(
                    (x, y), 1, 1,
                    linewidth=0.3,
                    edgecolor='#404040',
                    facecolor=color
                )
                self.ax.add_patch(rect)
                self.cell_patches[(x, y)] = rect
        
        # Set axis limits with padding
        self.ax.set_xlim(-0.5, self.map_width + 0.5)
        self.ax.set_ylim(-0.5, self.map_height + 0.5)
        self.ax.set_aspect('equal')
        
        # Add grid lines
        self.ax.set_xticks(range(0, self.map_width + 1, 5))
        self.ax.set_yticks(range(0, self.map_height + 1, 5))
        self.ax.grid(True, alpha=0.3, linestyle='--')
        
        # Redraw all existing bots
        for bot_id, bot_data in self.bots.items():
            self._draw_bot(bot_id, bot_data['coord'], bot_data['color'])
        
        # Add legend
        self._add_legend()
        
        self._refresh_plot()
    
    def _add_legend(self):
        """Add a legend explaining colors."""
        legend_elements = [
            patches.Patch(facecolor='#A0A0A0', edgecolor='black', label='Traversable (1)'),
            patches.Patch(facecolor='#8B4513', edgecolor='black', label='2-way Split (2)'),
            patches.Patch(facecolor='#FF69B4', edgecolor='black', label='3-way Split (3)'),
            patches.Patch(facecolor='#E6F3FF', edgecolor='black', label='Visited'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Leader'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Follower'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=8)
    
    def visual_callback(self, msg):
        """Handle incoming visual update messages."""
        self.get_logger().info(f"Visual update: type={msg.type}, bot_id={msg.bot_id}, color={msg.color}, coord=({msg.coord_x}, {msg.coord_y})")
        
        with self.plot_lock:
            if msg.type == "new_registration":
                self._handle_new_registration(msg)
            elif msg.type == "color_change":
                self._handle_color_change(msg)
            elif msg.type == "update_position":
                self._handle_position_update(msg)
            else:
                self.get_logger().warn(f"Unknown visual update type: {msg.type}")
    
    def _handle_new_registration(self, msg):
        """Handle new bot registration."""
        bot_id = msg.bot_id
        coord = (msg.coord_x, msg.coord_y)
        color = msg.color
        
        self.get_logger().info(f"Registering new bot {bot_id} at {coord} with color {color}")
        
        # Store bot data
        self.bots[bot_id] = {
            'coord': coord,
            'color': color,
            'circle': None,
            'label': None
        }
        
        # Draw the bot if map is initialized
        if self.map_initialized:
            self._draw_bot(bot_id, coord, color)
            self._refresh_plot()
    
    def _handle_color_change(self, msg):
        """Handle bot color change."""
        bot_id = msg.bot_id
        new_color = msg.color
        
        # If bot not registered yet, register it first
        if bot_id not in self.bots:
            self.get_logger().info(f"Bot {bot_id} not found, registering it now...")
            self.bots[bot_id] = {
                'coord': (msg.coord_x, msg.coord_y),
                'color': new_color,
                'circle': None,
                'label': None
            }
            if self.map_initialized:
                self._draw_bot(bot_id, (msg.coord_x, msg.coord_y), new_color)
                self._refresh_plot()
            return
        
        self.get_logger().info(f"Bot {bot_id} color changed to {new_color}")
        
        # Update stored color
        self.bots[bot_id]['color'] = new_color
        
        # Remove old circle and label
        self._remove_bot_graphics(bot_id)
        
        if self.map_initialized:
            coord = self.bots[bot_id]['coord']
            self._draw_bot(bot_id, coord, new_color)
            self._refresh_plot()
    
    def _handle_position_update(self, msg):
        """Handle bot position update."""
        bot_id = msg.bot_id
        new_coord = (msg.coord_x, msg.coord_y)
        
        # If bot not registered yet, register it first
        if bot_id not in self.bots:
            self.get_logger().info(f"Bot {bot_id} not found, registering it now...")
            self.bots[bot_id] = {
                'coord': new_coord,
                'color': msg.color,
                'circle': None,
                'label': None
            }
            if self.map_initialized:
                self._draw_bot(bot_id, new_coord, msg.color)
                self._refresh_plot()
            return
        
        old_coord = self.bots[bot_id]['coord']
        self.get_logger().info(f"Bot {bot_id} moved from {old_coord} to {new_coord}")
        
        # Mark old cell as visited
        self.visited_cells.add(old_coord)
        
        # Update the old cell's color to visited
        if old_coord in self.cell_patches:
            self.cell_patches[old_coord].set_facecolor(self.visited_color)
        
        # Update stored coord
        self.bots[bot_id]['coord'] = new_coord
        
        # Remove old circle and label
        self._remove_bot_graphics(bot_id)
        
        if self.map_initialized:
            color = self.bots[bot_id]['color']
            self._draw_bot(bot_id, new_coord, color)
            self._refresh_plot()
    
    def _remove_bot_graphics(self, bot_id):
        """Remove the circle and label for a bot."""
        if bot_id in self.bots:
            if self.bots[bot_id]['circle'] is not None:
                try:
                    self.bots[bot_id]['circle'].remove()
                except:
                    pass
                self.bots[bot_id]['circle'] = None
            if self.bots[bot_id]['label'] is not None:
                try:
                    self.bots[bot_id]['label'].remove()
                except:
                    pass
                self.bots[bot_id]['label'] = None
    
    def _draw_bot(self, bot_id, coord, color):
        """Draw a bot as a circle at the specified coordinate."""
        # Map color names to matplotlib colors with better visibility
        color_map = {
            'white': '#FFFFFF',
            'blue': '#1E90FF',    # Dodger blue for leaders
            'green': '#32CD32',   # Lime green for followers
            'yellow': '#FFD700',  # Gold for explorers
            'red': '#FF4500',     # Orange-red for returning
        }
        
        plot_color = color_map.get(color.lower(), color)
        
        # Create circle at cell center - larger and more visible
        circle = plt.Circle(
            (coord[0] + 0.5, coord[1] + 0.5),  # Center of the cell
            0.4,  # Larger radius for visibility
            color=plot_color,
            ec='black',
            linewidth=2,
            zorder=10  # Draw on top of map
        )
        
        self.ax.add_patch(circle)
        self.bots[bot_id]['circle'] = circle
        
        # Add bot ID label with better styling
        label = self.ax.text(
            coord[0] + 0.5, 
            coord[1] + 0.5,
            str(bot_id),
            ha='center',
            va='center',
            fontsize=9,
            fontweight='bold',
            color='black' if color.lower() in ['white', 'yellow'] else 'white',
            zorder=11
        )
        self.bots[bot_id]['label'] = label
    
    def _refresh_plot(self):
        """Refresh the matplotlib plot."""
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)  # Small pause to allow GUI update


def main(args=None):
    rclpy.init(args=args)
    
    visual_node = VisualNode()
    
    try:
        # Use a timer to periodically update matplotlib
        def spin_with_plot():
            while rclpy.ok():
                rclpy.spin_once(visual_node, timeout_sec=0.1)
                plt.pause(0.01)  # Keep matplotlib responsive
        
        spin_with_plot()
        
    except KeyboardInterrupt:
        pass
    finally:
        visual_node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()