#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from collections import deque
import numpy as np
import math
import time

class GPSOdomFilterNode(Node):
    def __init__(self):
        super().__init__('gps_odom_filter_node')
        
        # Parameters
        self.declare_parameter('window_size', 10)
        self.window_size = self.get_parameter('window_size').value
        
        self.declare_parameter('max_jump_distance', 3.0)  # meters
        self.max_jump_distance = self.get_parameter('max_jump_distance').value
        
        self.declare_parameter('stationary_velocity_threshold', 0.05)  # m/s
        self.stationary_velocity_threshold = self.get_parameter('stationary_velocity_threshold').value
        
        self.declare_parameter('stationary_window', 1.0)  # seconds
        self.stationary_window = self.get_parameter('stationary_window').value
        
        # Create a buffer for recent GPS fixes
        self.lat_buffer = deque(maxlen=self.window_size)
        self.lon_buffer = deque(maxlen=self.window_size)
        self.alt_buffer = deque(maxlen=self.window_size)
        gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        # Variables for odometry-based movement detection
        self.is_stationary = False
        self.stationary_start_time = None
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        
        # Static position when stationary
        self.static_position = None  # (lat, lon, alt) when robot is determined to be stationary
        
        # Previous valid position
        self.prev_lat = None
        self.prev_lon = None
        
        # Publishers and subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile=gps_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Update this to match your odometry topic
            self.odom_callback,
            10)
        
        self.filtered_gps_pub = self.create_publisher(
            NavSatFix,
            '/filtered_gps',
            10)
        
        # Timer for checking stationary status
        self.create_timer(0.1, self.check_stationary_status)
        
        self.get_logger().info('GPS-Odometry Filter Node initialized')
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate the great circle distance between two points in meters"""
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
        c = 2 * np.arcsin(np.sqrt(a))
        r = 6371000  # Radius of earth in meters
        return c * r
    
    def odom_callback(self, msg):
        # Extract linear and angular velocities from odometry
        self.current_linear_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2 + 
            msg.twist.twist.linear.z**2
        )
        self.current_angular_velocity = math.sqrt(
            msg.twist.twist.angular.x**2 + 
            msg.twist.twist.angular.y**2 + 
            msg.twist.twist.angular.z**2
        )
    
    def check_stationary_status(self):
        # Check if robot is currently stationary based on velocity thresholds
        is_currently_stationary = (
            self.current_linear_velocity < self.stationary_velocity_threshold and
            self.current_angular_velocity < 0.05  # Angular velocity threshold in rad/s
        )
        
        current_time = time.time()
        
        # State machine for detecting stationary periods
        if is_currently_stationary and not self.is_stationary:
            # Just became stationary
            if self.stationary_start_time is None:
                self.stationary_start_time = current_time
            elif (current_time - self.stationary_start_time) > self.stationary_window:
                # Been stationary long enough to consider truly stationary
                self.is_stationary = True
                # Average the buffer to get a stable position
                if len(self.lat_buffer) > 0:
                    self.static_position = (
                        np.median(self.lat_buffer),
                        np.median(self.lon_buffer),
                        np.median(self.alt_buffer)
                    )
                    self.get_logger().info(f"Robot is now stationary at: {self.static_position[0]:.7f}, {self.static_position[1]:.7f}")
        elif not is_currently_stationary:
            # Moving again
            self.is_stationary = False
            self.stationary_start_time = None
            self.static_position = None
    
    def gps_callback(self, msg):
        # Store the current GPS fix
        current_lat = msg.latitude
        current_lon = msg.longitude
        current_alt = msg.altitude
        
        # Initialize filtered message from the incoming message
        filtered_msg = NavSatFix()
        filtered_msg.header = msg.header
        filtered_msg.position_covariance = msg.position_covariance
        filtered_msg.position_covariance_type = msg.position_covariance_type
        
        # Different filtering logic based on whether the robot is moving or stationary
        if self.is_stationary and self.static_position is not None:
            # If stationary, use the stored static position
            filtered_msg.latitude = self.static_position[0]
            filtered_msg.longitude = self.static_position[1]
            filtered_msg.altitude = self.static_position[2]
            
            self.get_logger().debug(f'Robot stationary - Using fixed position')
        else:
            # If moving or recently started moving
            # Check if this is the first fix or if it's within acceptable distance
            if self.prev_lat is None or self.haversine_distance(
                    self.prev_lat, self.prev_lon, current_lat, current_lon) < self.max_jump_distance:
                
                # Add to buffer
                self.lat_buffer.append(current_lat)
                self.lon_buffer.append(current_lon)
                self.alt_buffer.append(current_alt)
                
                # Update previous position
                self.prev_lat = current_lat
                self.prev_lon = current_lon
                
                # If we have enough samples, use filtered position
                if len(self.lat_buffer) >= 3:  # Need at least 3 samples for a reasonable filter
                    filtered_msg.latitude = np.median(self.lat_buffer)
                    filtered_msg.longitude = np.median(self.lon_buffer)
                    filtered_msg.altitude = np.median(self.alt_buffer)
                else:
                    # Not enough samples yet, use current position
                    filtered_msg.latitude = current_lat
                    filtered_msg.longitude = current_lon
                    filtered_msg.altitude = current_alt
                
                self.get_logger().debug(f'Robot moving - Using filtered position')
            else:
                # Too far from previous position, likely a GPS jump
                self.get_logger().warn(f'Rejected GPS fix: too far from previous position')
                
                # Use the most recent valid position
                if len(self.lat_buffer) > 0:
                    filtered_msg.latitude = np.median(self.lat_buffer)
                    filtered_msg.longitude = np.median(self.lon_buffer)
                    filtered_msg.altitude = np.median(self.alt_buffer)
                else:
                    # No valid previous position, reluctantly use current
                    filtered_msg.latitude = current_lat
                    filtered_msg.longitude = current_lon
                    filtered_msg.altitude = current_alt
        
        # Publish the filtered message
        self.filtered_gps_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSOdomFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()