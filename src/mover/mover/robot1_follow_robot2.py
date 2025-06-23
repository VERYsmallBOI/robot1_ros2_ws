#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class FollowRobot(Node):
    def __init__(self):
        super().__init__('robot1_follow_robot2')

        self.robot2_pose = None
        self.robot1_pose = None

        self.subscription1 = self.create_subscription(
            Odometry,
            '/robot2/odom',  # Now robot2 is the target
            self.robot2_odom_callback,
            10)

        self.subscription2 = self.create_subscription(
            Odometry,
            '/robot1/odom',  # robot1 is the follower
            self.robot1_odom_callback,
            10)

        self.publisher = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def robot2_odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot2_pose = (pos.x, pos.y, yaw)

    def robot1_odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot1_pose = (pos.x, pos.y, yaw)

    def control_loop(self):
        if not self.robot1_pose or not self.robot2_pose:
            return

        x1, y1, yaw1 = self.robot1_pose
        x2, y2, yaw2 = self.robot2_pose

        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx ** 2 + dy ** 2)

        target_angle = math.atan2(dy, dx)
        angle_diff = normalize_angle(target_angle - yaw1)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if distance > 0.05:
            msg.twist.linear.x = 0.15
            msg.twist.angular.z = 0.5 * angle_diff
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
