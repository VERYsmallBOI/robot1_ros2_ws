#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import math
import transforms3d.euler as euler  # from `transforms3d` package

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class GoToPointStamped(Node):
    def __init__(self):
        super().__init__('go_to_point_stamped')
        self.publisher_ = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.target_x = -.50
        self.target_y = .750
        self.odom_ready = False
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'rotate'

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.w, q.x, q.y, q.z]
        _, _, self.theta = euler.quat2euler(quat)
        self.odom_ready = True

    def control_loop(self):
     if not self.odom_ready:
         return
 
     dx = self.target_x - self.x
     dy = self.target_y - self.y
     angle_to_goal = math.atan2(dy, dx)
     angle_diff = normalize_angle(angle_to_goal - self.theta)
 
     distance = math.sqrt(dx**2 + dy**2)
 
     # Log current position and distance
     self.get_logger().info(f'Current Position: x={self.x:.2f}, y={self.y:.2f}, Distance to Goal: {distance:.2f}')
 
     msg = TwistStamped()
     msg.header.stamp = self.get_clock().now().to_msg()
 
     if self.state == 'rotate':
         if abs(angle_diff) > 0.025:
             msg.twist.angular.z = 0.1 if angle_diff > 0 else -0.1
         else:
             self.state = 'move'
 
     elif self.state == 'move':
         if distance > 0.05:
             msg.twist.linear.x = 0.2
         else:
             self.get_logger().info("Arrived at target!")
             msg.twist.linear.x = 0.0
             self.state = 'stop'
 
     self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPointStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
