import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

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

        plt.figure()
        plt.imshow(map_array, cmap='gray', origin='lower')
        plt.title('Occupancy Grid Map')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.colorbar()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()

    try:
        rclpy.spin(map_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        map_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
