#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class GoToZeroStamped(Node):
    def __init__(self):
        super().__init__('go_to_zero_stamped')

        self.cmd_pub = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_reached = False

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
        if self.target_reached:
            return

        dx = -self.x
        dy = -self.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)

        yaw_error = angle_to_target - self.yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize

        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        if abs(yaw_error) > 0.1:
            cmd_msg.twist.angular.z = 0.5 * yaw_error
        elif distance > 0.05:
            cmd_msg.twist.linear.x = 0.3 * distance
        elif abs(self.yaw) > 0.1:
            cmd_msg.twist.angular.z = -0.5 * self.yaw
        else:
            self.get_logger().info('Reached (0,0,0)')
            self.cmd_pub.publish(TwistStamped())  # Stop
            self.target_reached = True
            return

        self.cmd_pub.publish(cmd_msg)

def main():
    rclpy.init()
    node = GoToZeroStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
