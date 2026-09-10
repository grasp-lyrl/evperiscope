#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Twist


class PoseToOdomRepublisher(Node):
    """Node that republishes PoseStamped messages as Odometry messages."""
    def __init__(self):
        super().__init__('pose_to_odom_republisher')
        self.declare_parameter('input_topic', '/pose')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        self.subscription = self.create_subscription(
            PoseStamped, input_topic, self.pose_callback, 10)
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.get_logger().info(f'Republishing {input_topic} as {output_topic}')
    
    def pose_callback(self, msg: PoseStamped):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'ugv_map'
        odom.child_frame_id = self.child_frame_id
        odom.pose = PoseWithCovariance()
        odom.pose.pose = msg.pose
        odom.pose.covariance = [0.0] * 36
        odom.twist = TwistWithCovariance()
        odom.twist.twist = Twist()
        odom.twist.covariance = [0.0] * 36
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    republisher = PoseToOdomRepublisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
