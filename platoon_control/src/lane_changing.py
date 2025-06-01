#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AdaptivePathPlanner(Node):
    def __init__(self):
        
        super().__init__("path_planner")
        self.cmd_vel_pub = self.create_publisher(Twist, "/wafflepi1/cmd_vel", 10)
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  # Ensure best effort delivery

        # Subscriber for laser scan data with compatible QoS
        self.scan_subscription = self.create_subscription(
            LaserScan, 
            '/wafflepi1/scan', 
            self.scan_callback, 
            qos_profile  # Apply the QoS profile
        )

        # super().__init__("path_planner")
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile)
        
        # Subscriber for odometry data to track position and orientation
        self.odom_subscriber = self.create_subscription(
            Odometry, '/wafflepi1/odom', self.odom_callback, 10
        )
        
        # Subscriber for laser scan data to detect obstacles
        # self.scan_subscription = self.create_subscription(
        #     LaserScan, '/scan', self.scan_callback, qos_profile
        # )
        
        # Goal parameters
        self.goal_x = 4.0
        self.goal_y = 0.0
        self.goal_threshold = 0.1 

        # control parameters
        self.linear_kp = 0.8
        self.angular_kp = 2.0
        self.obstacle_threshold = 0.5  # Safe distance in meters
        self.obstacle_detected = False
        self.count = 0
        self.lane_width = 0.6   # To determine how close the robot should be while overtaking obstacles

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_reached = False

    def scan_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec

        # Calculate indices for 0°, 90°, 270°, and 360°
        index_0_deg = int((0 - msg.angle_min) / msg.angle_increment)
        # index_90_deg = int((math.radians(90) - msg.angle_min) / msg.angle_increment)
        index_90_deg = int((math.radians(90) - msg.angle_min) / msg.angle_increment)
        index_180_deg = int((math.radians(180) - msg.angle_min) / msg.angle_increment)
        index_270_deg = int((math.radians(270) - msg.angle_min) / msg.angle_increment)
        index_360_deg = int((math.radians(360) - msg.angle_min) / msg.angle_increment)

        # Extract ranges for 0° to 90° and 270° to 360°
        self.ranges_0_to_90 = msg.ranges[index_0_deg:index_90_deg + 1]
        self.ranges_90_to_180 = msg.ranges[index_90_deg:index_180_deg + 1]
        self.ranges_180_to_270 = msg.ranges[index_180_deg:index_270_deg + 1]
        self.ranges_270_to_360 = msg.ranges[index_270_deg:index_360_deg + 1]
        self.all_ranges = list(msg.ranges)
        self.get_logger().info(f"all_ranges= {len(self.all_ranges)}")

        # Log only the range values for these two segments
        # self.get_logger().info(
        #     f"Time: {timestamp} sec\nRanges 0° to 90°: {self.ranges_0_to_90}\nRanges 270° to 360°: {self.ranges_270_to_360}"
        # )
        
        # self.get_logger().info(
        #     f"All ranges={self.all_ranges}"
        # )
        
        # Check if any value in either range is below the obstacle threshold
        self.get_logger().info(f"length: {len(self.all_ranges)}")
        self.road_left_view = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*len(self.all_ranges)/(2*math.pi))]) if dist <= 0.7]
        self.road_right_view = [i for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*len(self.all_ranges)/(2*math.pi))):]) if dist <= 0.7]
        # self.road_right_view = [i for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/(math.pi))):]) if dist <= 1.0]
        self.road_block = 1
        self.get_logger().info(
            f"road_left={self.road_left_view}"
        )
        self.get_logger().info(
            f"road_right={self.road_right_view}"
        )
        if len(self.road_left_view) == 0 and len(self.road_right_view) == 0:
            self.road_block = 0
        self.obstacle_detected = (
            any(distance < self.obstacle_threshold for distance in self.road_left_view)
             or
            any(distance < self.obstacle_threshold for distance in self.road_right_view)
            # distance < self.obstacle_threshold for distance in self.all_ranges
        )
        self.obstacle_detected = self.road_block
        self.get_logger().info(f"road_block: {self.road_block}")
        self.get_logger().info(f"obstacle_detected: {self.obstacle_detected}")

    
    def odom_callback(self, msg):
        # Update current position and orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Check if goal is reached
        distance_to_goal = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        self.get_logger().info(f"dist_to_goal: {distance_to_goal}")
        if distance_to_goal <= self.goal_threshold:
            self.get_logger().info(f"Case #Reached Goal")
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info("Goal reached!")
            return
        
        # Move toward goal, adapting to obstacles if detected
        if not self.goal_reached:
            self.move_toward_goal(distance_to_goal)

    def move_toward_goal(self, distance_to_goal):
        required_yaw = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
        self.get_logger().info(f"Curent yaw: {self.current_yaw}")
        self.get_logger().info(f"odomdata: {self.current_x:.2f}, {self.current_y:.2f}")
        # self.road_left_view = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*180/math.pi)]) if dist <= 2.0]
        # self.road_right_view = [i for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*180/math.pi)):]) if dist <= 2.0]
        # self.road_block = 1
        # if len(self.road_left_view) != 0 and len(self.road_right_view) != 0:
        #     road_block = 0
        if self.obstacle_detected:
            self.get_logger().info(f"Case #0")
            self.get_logger().info(f"Obstacle detected, adapting path...")

            # Determine the clearance angles for both sides
            # self.left_clearance = [i for i, dist in enumerate(self.all_ranges[:len(self.all_ranges) // 4]) if dist <= 2.0]
            # self.right_clearance = [len(self.all_ranges) // 4 - i - 1 for i, dist in enumerate(self.all_ranges[-len(self.all_ranges) // 4:]) if dist <= 2.0]
            self.left_clearance = [i for i, dist in enumerate(self.all_ranges[:int(len(self.all_ranges)//4) - int(self.current_yaw*len(self.all_ranges)/(2*math.pi))]) if dist <= 1.0]
            self.right_clearance = [len(self.all_ranges) // 4 + int(self.current_yaw*len(self.all_ranges)/(2*math.pi)) - i - 1 for i, dist in enumerate(self.all_ranges[-(int(len(self.all_ranges)//4) + int(self.current_yaw*len(self.all_ranges)/(2*math.pi))):]) if dist <= 1.0]
            self.get_logger().info(
                f"right_clearance={self.right_clearance}"
            )
            self.get_logger().info(
                f"left_clearance={self.left_clearance}"
            )
            # self.get_logger().info(
            #     f"all_ranges={self.all_ranges}"
            # )
            # Check left and right clearance limits
            self.left_deg = self.left_clearance[-1] if self.left_clearance else 0
            # self.right_deg = int(len(self.all_ranges)/4) - self.right_clearance[-1] - 1 if self.right_clearance else 90
            self.right_deg = self.right_clearance[0] if self.right_clearance else 0
            self.get_logger().info(f"left_deg = {self.left_deg} right_deg = {self.right_deg}")

            # Move forward along the obstacle once aligned
            if self.left_deg <= self.right_deg:
                required_yaw = (self.lane_width - self.current_y)*(math.pi/2)
                yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
                self.get_logger().info(f"Case #1")
                self.get_logger().info(f"required_yaw: {required_yaw}")
                self.get_logger().info(f"current_yaw: {self.current_yaw}")
                self.get_logger().info(f"yaw_error: {yaw_error}")
                linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
                angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
                self.get_logger().info(f"angular_speed: {angular_speed}")
                self.get_logger().info(f"linear_vel: {linear_speed}")
                self.publish_velocity(linear_speed, angular_speed)
            else:
                self.get_logger().info(f"Case #1R")
                required_yaw = (- self.lane_width - self.current_y)*(math.pi/2)
                self.get_logger().info(f"required_yaw: {required_yaw}")
                self.get_logger().info(f"current_yaw: {self.current_yaw}")
                yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
                linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
                self.get_logger().info(f"yaw_error: {yaw_error}")
                angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
                self.get_logger().info(f"angular_speed: {angular_speed}")
                self.get_logger().info(f"linear_vel: {linear_speed}")
                self.publish_velocity(linear_speed, angular_speed)
                
        else:
            self.get_logger().info(f"Case #100")
            required_yaw = (0.0 - self.current_y)*(math.pi/2)
            yaw_error = math.atan2(math.sin(required_yaw - self.current_yaw), math.cos(required_yaw - self.current_yaw))
            self.get_logger().info(f"required_yaw: {required_yaw}")
            self.get_logger().info(f"yaw_error: {yaw_error}")
            self.get_logger().info(f"current_yaw: {self.current_yaw}")
            linear_speed = min(self.linear_kp * distance_to_goal, 0.1)
            angular_speed = max(-0.8, min(self.angular_kp * yaw_error, 0.8))
            # angular_speed = 0.0
            self.get_logger().info(f"angular_speed: {angular_speed}")
            self.get_logger().info(f"linear_vel: {linear_speed}")
            self.publish_velocity(linear_speed, angular_speed)


    def publish_velocity(self, speed, turn):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = turn
        self.cmd_vel_pub.publish(vel_msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)  # Stop the robot

def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
