#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

reached = False

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_1 = self.create_publisher(Float64MultiArray, '/left_joint_1_position_controller/commands', 10)
        self.publisher_2 = self.create_publisher(Float64MultiArray, '/left_joint_2_position_controller/commands', 10)
        self.publisher_3 = self.create_publisher(Float64MultiArray, '/right_joint_1_position_controller/commands', 10)
        self.publisher_4 = self.create_publisher(Float64MultiArray, '/right_joint_2_position_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        
        # Define different target positions
        self.poses = {
            'holding': [0.288, 0.832, 0.288, 0.832],
            'idle': [1.614, -3.142, 1.614, -3.142],
            'non_holding': [0.254, 0.358, 0.254, 0.358]
        }
        
        # Initialize the current positions and set initial target positions
        self.current_positions = [0, 0, 0, 0]
        self.target_positions = self.poses['non_holding']  # Default initial pose
        self.step = 0.02

    def timer_callback(self):

        global reached

        # Update positions slowly
        for i in range(len(self.current_positions)):
            if self.current_positions[i] < self.target_positions[i]:
                self.current_positions[i] += self.step
                if self.current_positions[i] > self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]
            elif self.current_positions[i] > self.target_positions[i]:
                self.current_positions[i] -= self.step
                if self.current_positions[i] < self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]

        msg = Float64MultiArray()
        msg.data = [self.current_positions[0]]
        self.publisher_1.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        msg = Float64MultiArray()
        msg.data = [self.current_positions[1]]
        self.publisher_2.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        msg = Float64MultiArray()
        msg.data = [self.current_positions[2]]
        self.publisher_3.publish(msg)

        msg = Float64MultiArray()
        msg.data = [self.current_positions[3]]
        self.publisher_4.publish(msg)
        
        # Check if all joints have reached their target positions within tolerance
        if all(abs(cp - tp) < self.step/2 for cp, tp in zip(self.current_positions, self.target_positions)):
            self.get_logger().info('Reached target positions')
            reached = True
            # self.timer.cancel()  # Stop the timer
            # self.destroy_node()  # Destroy the node to exit

def main(args=None):

    global reached

    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        while rclpy.ok():
            # Prompt user for input
            print("Available poses: holding, idle, non_holding")
            pose_input = input("Enter desired pose ('q' to quit): ")

            if pose_input.lower() == 'q':
                break  # Exit loop and shutdown

            print(list(node.poses.keys()))

            if pose_input in list(node.poses.keys()):
                node.target_positions = node.poses[pose_input]
                while not reached:
                    rclpy.spin_once(node)
                reached = False
            else:
                print("Invalid pose. Please enter one of: holding, idle, non_holding")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
