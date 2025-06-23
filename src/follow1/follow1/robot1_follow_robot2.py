import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import tf_transformations


class RobotFollower(Node):
    def __init__(self):
        super().__init__('robot_follower')

        self.robot1_ns = 'robot1'
        self.robot2_ns = 'robot2'

        # Parameters for control
        self.min_distance = 0.3  # meters (minimum distance to maintain)
        self.max_distance = 0.5  # meters (start slowing down when approaching this)
        self.max_linear_speed = 0.2  # m/s
        self.max_angular_speed = 0.5  # rad/s
        self.angle_threshold = 0.2  # rad (about 11.5 degrees) before we start moving

        self.sub_odom1 = self.create_subscription(
            Odometry,
            f'/{self.robot1_ns}/odom',
            self.robot1_odom_callback,
            10)

        self.sub_odom2 = self.create_subscription(
            Odometry,
            f'/{self.robot2_ns}/odom',
            self.robot2_odom_callback,
            10)

        self.pub_cmd_vel = self.create_publisher(
            TwistStamped,
            f'/{self.robot1_ns}/cmd_vel',
            10)

        self.robot1_pose = None
        self.robot2_pose = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def robot1_odom_callback(self, msg):
        self.robot1_pose = self.extract_pose(msg, apply_yaw_offset=True)
        self.get_logger().info(f'Robot1 pose - X: {self.robot1_pose[0]:.2f}, Y: {self.robot1_pose[1]:.2f}, Yaw: {math.degrees(self.robot1_pose[2]):.2f}°')

    def robot2_odom_callback(self, msg):
        self.robot2_pose = self.extract_pose(msg, apply_yaw_offset=False)
        self.get_logger().info(f'Robot2 pose - X: {self.robot2_pose[0]:.2f}, Y: {self.robot2_pose[1]:.2f}, Yaw: {math.degrees(self.robot2_pose[2]):.2f}°')

    def extract_pose(self, msg, apply_yaw_offset=False):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w])

        if apply_yaw_offset:
            yaw -= math.pi / 2  # Apply 90 degree offset if needed

        return (position.x, position.y, yaw)

    def control_loop(self):
        if self.robot1_pose is None or self.robot2_pose is None:
            self.get_logger().warn('Waiting for both robot poses...')
            return

        x1, y1, yaw1 = self.robot1_pose
        x2, y2, yaw2 = self.robot2_pose

        # Calculate relative position
        dx = x2 - x1
        dy = y2 - y1
        
        # Transform to robot1's frame
        robot_frame_dx = dx * math.cos(yaw1) + dy * math.sin(yaw1)
        robot_frame_dy = -dx * math.sin(yaw1) + dy * math.cos(yaw1)
        
        distance = math.hypot(robot_frame_dx, robot_frame_dy)
        target_yaw = math.atan2(robot_frame_dy, robot_frame_dx)
        angle_diff = self.normalize_angle(target_yaw)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot1_ns

        self.get_logger().info(f'Relative position - X: {robot_frame_dx:.2f}, Y: {robot_frame_dy:.2f}')
        self.get_logger().info(f'Distance: {distance:.2f}m, Angle diff: {math.degrees(angle_diff):.2f}°')

        if distance < self.min_distance:
            # Too close - stop or move backward
            msg.twist.linear.x = max(-0.05, -0.1 * (self.min_distance - distance))
            msg.twist.angular.z = 0.0
            self.get_logger().warn(f'Too close! Distance: {distance:.2f}m. Holding or moving back.')
        elif distance > self.max_distance:
            # Far enough - approach
            if abs(angle_diff) > self.angle_threshold:
                # Turn first to face the target
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = self.max_angular_speed * (1.0 if angle_diff > 0 else -1.0)
                self.get_logger().warn(f'Aligning to target. Angle diff: {math.degrees(angle_diff):.2f}°')
            else:
                # Move forward while making small corrections
                speed_factor = min(1.0, (distance - self.min_distance) / (self.max_distance - self.min_distance))
                msg.twist.linear.x = self.max_linear_speed * speed_factor
                msg.twist.angular.z = 2.0 * angle_diff  # Proportional control for angle
                self.get_logger().info(f'Approaching target. Speed: {msg.twist.linear.x:.2f}m/s')
        else:
            # In the sweet spot - maintain position with small corrections
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.5 * angle_diff
            self.get_logger().info(f'Maintaining position. Small angle correction: {math.degrees(msg.twist.angular.z):.2f}°/s')

        # Ensure we don't exceed speed limits
        msg.twist.linear.x = max(min(msg.twist.linear.x, self.max_linear_speed), -self.max_linear_speed)
        msg.twist.angular.z = max(min(msg.twist.angular.z, self.max_angular_speed), -self.max_angular_speed)

        self.pub_cmd_vel.publish(msg)

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RobotFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
