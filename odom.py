#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# ROS 2 message imports
from nav_msgs.msg import Odometry
from geometry_dict.msg import TransformStamped, Quaternion
from std_msgs.msg import Int32
import tf2_ros

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # -----------------------------
        # ROBOT CONFIGURATION CONSTANTS
        # -----------------------------
        self.TICKS_PER_REV = 2000.0  # Adjust for your encoder (quadrature)
        self.WHEEL_RADIUS = 0.0325    # In meters (e.g., 65mm diameter)
        self.WHEEL_TRACK = 0.160      # Distance between wheels in meters
        
        # Distance scaled per individual tick
        self.METERS_PER_TICK = (2.0 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV

        # Robot Pose State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0 # Theta orientation in radians

        # Encoder tracking
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Timing tracking
        self.last_time = self.get_clock().now()

        # ROS Subscriptions (From your motor/encoder driver)
        self.sub_left = self.create_subscription(Int32, 'left_encoder_ticks', self.left_cb, 10)
        self.sub_right = self.create_subscription(Int32, 'right_encoder_ticks', self.right_cb, 10)

        # ROS Publishers and Broadcasters
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Run calculation loop at 20 Hz
        self.timer = self.create_timer(0.05, self.update_odometry)

    def left_cb(self, msg):
        self.left_ticks = msg.data

    def right_cb(self, msg):
        self.right_ticks = msg.data

    def quaternion_from_euler(self, ai, aj, ak):
        """Helper to convert Euler Z (yaw) to Quaternion."""
        cy = math.cos(ak * 0.5)
        sy = math.sin(ak * 0.5)
        cp = math.cos(aj * 0.5)
        sp = math.sin(aj * 0.5)
        cr = math.cos(ai * 0.5)
        sr = math.sin(ai * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        # 1. Calculate change in ticks since last loop
        delta_left = self.left_ticks - self.last_left_ticks
        delta_right = self.right_ticks - self.last_right_ticks

        # Save current ticks for the next iteration
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

        # 2. Convert ticks to actual real-world distance travel
        dist_left = delta_left * self.METERS_PER_TICK
        dist_right = delta_right * self.METERS_PER_TICK

        # 3. Apply Differential Drive Math
        d_left_right = dist_right - dist_left
        d_center = (dist_left + dist_right) / 2.0
        d_theta = d_left_right / self.WHEEL_TRACK

        # 4. Integrate into Global Coordinates
        # (Using midpoint integration for accuracy)
        self.x += d_center * math.cos(self.th + (d_theta / 2.0))
        self.y += d_center * math.sin(self.th + (d_theta / 2.0))
        self.th += d_theta

        # Normalize theta between -pi and pi
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Calculate Velocities
        v_linear = d_center / dt
        v_angular = d_theta / dt

        # Convert Orientation to Quaternion
        odom_quat = self.quaternion_from_euler(0, 0, self.th)

        # 5. Broadcast the TF Transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(t)

        # 6. Publish the Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set the Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat

        # Set the Velocity
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = v_angular

        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
