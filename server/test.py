#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import logging

logging.basicConfig(level=logging.INFO)

class TestROSNode(Node):
    def __init__(self):
        super().__init__('test_map_server')
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        logging.info("Test ROSNode initialized and subscriptions created.")

    def map_callback(self, msg):
        logging.info("Map callback triggered.")

    def odom_callback(self, msg):
        logging.info("Odometry callback triggered.")

def main(args=None):
    rclpy.init(args=args)
    node = TestROSNode()
    logging.info("Test ROSNode started.")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
