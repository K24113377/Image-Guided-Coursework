#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_igtl_bridge.msg import Transform
from geometry_msgs.msg import PoseStamped

class MemberSubscriber(Node):
    def __init__(self):
        super().__init__('member_subscriber')
        self.latest_pose = None
        self.subscription = self.create_subscription(
            Transform,
            '/IGTL_TRANSFORM_IN',
            self.listener_callback,
            1
        )
        self.get_logger().info('MemberSubscriber node has started')

    def listener_callback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.transform.translation.x
        pose.pose.position.y = msg.transform.translation.y
        pose.pose.position.z = msg.transform.translation.z
        pose.pose.orientation = msg.transform.rotation
        self.latest_pose = pose
        self.get_logger().info(f"Received pose: {pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    subscriber = MemberSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
