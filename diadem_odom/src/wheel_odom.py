#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import math
from tf_transformations import quaternion_from_euler

class SkidSteerTicksToOdom(Node):
    def __init__(self):
        super().__init__('skid_steer_ticks_to_odom_node')
        
        # Parameters with defaults (adjust in launch file)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('track_width', 0.561),
                ('wheel_base', 0.406),
                ('wheel_radius', 0.127),
                ('ticks_per_revolution', 498.4),
                ('rotation_scale', 1.0),
                ('slip_threshold', 0.02),
                ('slip_compensation_factor', 0.5)
            ])
        
        # Load parameters
        self.track_width = self.get_parameter('track_width').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.slip_compensation = self.get_parameter('slip_compensation_factor').value

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks = None
        self.last_time = self.get_clock().now()

        # Configure QoS for odometry publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber with Best Effort QoS
        self.wheel_ticks_sub = self.create_subscription(
            Int32MultiArray,
            'wheel/ticks',
            self.wheel_ticks_callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        
        self.get_logger().info(f'''
Skid Steer Odometry Node Initialized:
- Track Width: {self.track_width:.3f}m
- Wheel Radius: {self.wheel_radius:.3f}m
- Ticks/Rev: {self.ticks_per_revolution}
- Slip Threshold: {self.slip_threshold:.3f}m
''')

    def wheel_ticks_callback(self, msg):
        current_time = self.get_clock().now()
        if len(msg.data) != 4:
            self.get_logger().error(f'Invalid wheel ticks length: {len(msg.data)}')
            return

        # Extract ticks in order: [front_left, back_left, front_right, back_right]
        fl, bl, fr, br = msg.data
        
        # Initialize previous ticks
        if self.last_ticks is None:
            self.last_ticks = [fl, bl, fr, br]
            self.last_time = current_time
            return
            
        # Calculate time delta
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        if dt <= 0:
            return

        # Calculate tick differences and convert to meters
        try:
            d_fl = self.ticks_to_meters(fl - self.last_ticks[0])
            d_bl = self.ticks_to_meters(bl - self.last_ticks[1])
            d_fr = self.ticks_to_meters(fr - self.last_ticks[2])
            d_br = self.ticks_to_meters(br - self.last_ticks[3])
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        # Detect and compensate for wheel slip
        d_left, slip_left = self.handle_slip(d_fl, d_bl, 'left')
        d_right, slip_right = self.handle_slip(d_fr, d_br, 'right')

        # Calculate linear and angular displacements
        d_center = (d_left + d_right) / 2.0
        wheel_diff = (d_right - d_left) 
        d_theta = (wheel_diff / self.track_width) * self.rotation_scale

        # Update pose
        self.integrate_motion(d_center, d_theta, dt)

        # Publish odometry and tf
        self.publish_odometry(current_time, d_center/dt, d_theta/dt)

        # Update previous values
        self.last_ticks = [fl, bl, fr, br]
        self.last_time = current_time

    def ticks_to_meters(self, ticks):
        if abs(ticks) > 10000:  # Handle possible encoder wrap/glitch
            raise ValueError(f'Abnormal tick difference detected: {ticks}')
        return (ticks / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

    def handle_slip(self, front, rear, side):
        """Apply slip compensation if front/rear difference exceeds threshold"""
        diff = abs(front - rear)
        if diff > self.slip_threshold:
            self.get_logger().debug(f'Slip detected on {side} side: {diff:.4f}m')
            avg = (front + rear) / 2.0
            return avg * (1 - self.slip_compensation), True
        return (front + rear) / 2.0, False

    def integrate_motion(self, d_center, d_theta, dt):
        """Update robot pose using kinematic equations"""
        if abs(d_theta) < 1e-6:  # Straight motion
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:  # Arc motion
            radius = d_center / d_theta
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        self.theta += d_theta
        self.theta = self.normalize_angle(self.theta)

    def normalize_angle(self, angle):
        """Wrap angle to [-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def publish_odometry(self, current_time, vx, vth):
        # Create TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # self.tf_broadcaster.sendTransform(t)

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        
        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # Covariance matrices (adjust values based on your system's characteristics)
        odom.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # X, Y, Z, RotX, RotY, RotZ
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.3  # Higher uncertainty in yaw
        ]
        
        odom.twist.covariance = [
            0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.4  # Higher uncertainty in angular velocity
        ]

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerTicksToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()