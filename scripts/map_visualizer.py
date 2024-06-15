import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from threading import Thread

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.map_received = False

        # Initialize Tkinter
        self.root = tk.Tk()
        self.root.title("Occupancy Grid Map")

        # Create a figure and a canvas
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # Update Tkinter every 100 ms
        self.update_tkinter()

    def map_callback(self, msg):
        self.get_logger().info('Received map data')
        self.map_data = msg
        self.map_received = True
        self.plot_map()

    def plot_map(self):
        if not self.map_received:
            return
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        map_array = np.array(self.map_data.data).reshape((height, width))

        self.ax.clear()
        self.ax.imshow(map_array, cmap='gray', origin='lower')
        self.ax.set_title('Occupancy Grid Map')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.canvas.draw()

    def update_tkinter(self):
        self.root.update()
        self.root.after(100, self.update_tkinter)

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()

    ros_thread = Thread(target=rclpy.spin, args=(map_visualizer,))
    ros_thread.start()

    try:
        map_visualizer.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        map_visualizer.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()
