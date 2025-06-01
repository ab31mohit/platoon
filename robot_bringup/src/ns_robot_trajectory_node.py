#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class TrajectoryNode(Node):
    def __init__(self, namespace="", odom_topic="odom", trajectory_topic="trajectory"):
        # Use namespace if provided
        node_name = f'{namespace}_trajectory_node' if namespace else 'trajectory_node'
        super().__init__(node_name)
        
        # Get parameters
        self.declare_parameter('robot_name', namespace)
        self.declare_parameter('odom_topic', odom_topic)
        self.declare_parameter('trajectory_topic', trajectory_topic)
        
        robot_name = self.get_parameter('robot_name').value
        odom_topic = self.get_parameter('odom_topic').value
        trajectory_topic = self.get_parameter('trajectory_topic').value
        
        self.get_logger().info(f'Starting trajectory node for robot: {robot_name}')
        self.get_logger().info(f'Subscribing to odometry topic: {odom_topic}')
        self.get_logger().info(f'Publishing trajectory on topic: {trajectory_topic}')
        
        # Create publisher and subscriber with the specified topics
        self.path_pub = self.create_publisher(Path, trajectory_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        self.path = Path()
        self.path.header.frame_id = 'world'

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        # Publish the path
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()