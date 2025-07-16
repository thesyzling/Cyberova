#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPoseSetterNode(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(2.0)
        
        # Set initial pose
        self.set_initial_pose()
        
    def set_initial_pose(self):
        """Set the initial pose for SLAM"""
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        
        # Set initial position (robot starts at origin)
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        
        # Set initial orientation (facing forward)
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Set covariance matrix (diagonal)
        covariance = [0.0] * 36
        covariance[0] = 0.25    # x variance
        covariance[7] = 0.25    # y variance
        covariance[35] = 0.06854  # yaw variance
        initial_pose.pose.covariance = covariance
        
        # Publish initial pose multiple times to ensure delivery
        for i in range(5):
            self.initial_pose_pub.publish(initial_pose)
            self.get_logger().info(f'Published initial pose {i+1}/5')
            time.sleep(0.5)
            
        self.get_logger().info('Initial pose set successfully!')

def main(args=None):
    rclpy.init(args=args)
    
    node = InitialPoseSetterNode()
    
    # Shutdown after setting pose
    rclpy.shutdown()

if __name__ == '__main__':
    main() 