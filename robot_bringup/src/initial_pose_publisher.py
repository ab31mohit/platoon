#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import yaml
import os
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROBOT_NAMESPACE = os.environ.get('TURTLEBOT3_NAMESPACE', 'default_ns')  # Default to 'default_ns' if not set

class PoseInitializationPublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Create publisher for pose relocalization
        self.publisher = self.create_publisher(
            Point,
            'pose_relocalization',
            10
        )
        
        # Subscribe to odometry to ensure the odometry node is ready
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Flag to track if we've published the initial pose
        self.initial_pose_published = False
        
        # Wait a short time for the odometry node to initialize
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Get parameter file path
        try:
            param_dir = os.path.join(
                get_package_share_directory('robot_bringup'),
                'param',
                os.environ['TURTLEBOT3_MODEL'] + '.yaml'
            )
            
            # Read parameters from yaml file
            with open(param_dir, 'r') as file:
                self.params = yaml.safe_load(file)
                
            # Extract pose data from parameters
            self.initial_pose = (
                self.params.get(f'{ROBOT_NAMESPACE}', {})
                .get('initial_pose_publisher', {})
                .get('ros__parameters', {})
                .get('odometry', {})
                .get('initial_transform', {})
            )
            if not self.initial_pose:
                self.get_logger().warn('No initial pose found in parameters, using defaults')
                self.initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            else:
                self.get_logger().info(f"Loaded initial pose: x={float(self.initial_pose.get('x'))}, "
                           	f"y={float(self.initial_pose.get('y'))}, "
                           	f"yaw={float(self.initial_pose.get('yaw'))}")
    
        except Exception as e:
            self.get_logger().error(f'Failed to read parameters: {str(e)}')
            self.initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def odom_callback(self, msg):
        """Called when odometry messages are received"""
        if not self.initial_pose_published:
            self.publish_initial_pose()

    def timer_callback(self):
        """Backup timer to ensure pose gets published"""
        if not self.initial_pose_published:
            self.publish_initial_pose()

    def publish_initial_pose(self):
        """Publish the initial pose from parameters"""
        msg = Point()
        msg.x = float(self.initial_pose.get('x'))
        msg.y = float(self.initial_pose.get('y'))
        msg.z = float(self.initial_pose.get('yaw'))  # Using z for yaw

        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published initial pose: x={msg.x}, y={msg.y}, yaw={msg.z}'
        )

        self.initial_pose_published = True
        # Stop the timer after publishing
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PoseInitializationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
