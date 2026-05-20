#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
from pyproj import Transformer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import time
from collections import deque
import statistics
from tf_transformations import euler_from_quaternion

class GPSFilterNode(Node):
    def __init__(self):
        super().__init__('gps_filter_node')
        
        # Parameters
        self.declare_parameter('initial_collection_time', 20.0)
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('filtered_gps_topic', '/gps/filtered_fix')
        self.declare_parameter('debug_mode', True)
        
        # New parameters for fixed starting point
        self.declare_parameter('use_fixed_starting_point', True)  # Toggle to use fixed coordinates
        self.declare_parameter('fixed_latitude', 18.6171216)  # Your fixed latitude
        self.declare_parameter('fixed_longitude', 73.9098935)  # Your fixed longitude
        self.declare_parameter('reverse_direction', True)  # Toggle to reverse direction
        
        self.initial_collection_time = self.get_parameter('initial_collection_time').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.filtered_gps_topic = self.get_parameter('filtered_gps_topic').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Get fixed starting point parameters
        self.use_fixed_starting_point = self.get_parameter('use_fixed_starting_point').value
        self.fixed_latitude = self.get_parameter('fixed_latitude').value
        self.fixed_longitude = self.get_parameter('fixed_longitude').value
        self.reverse_direction = self.get_parameter('reverse_direction').value
        
        gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        if not self.use_fixed_starting_point:
            self.gps_sub = self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self.gps_callback,
                qos_profile=gps_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        
        # Publisher
        self.filtered_gps_pub = self.create_publisher(
            NavSatFix,
            self.filtered_gps_topic,
            10)
        
        # Variables for initial GPS data collection
        self.start_time = time.time()
        self.gps_collection_complete = False
        self.gps_data_buffer = deque(maxlen=1000)
        
        # Variables for odometry-based GPS
        self.initial_lat = None
        self.initial_lon = None
        self.initial_alt = 0.0
        self.initial_odom_x = None
        self.initial_odom_y = None
        self.initial_yaw = None
        
        # Current orientation
        self.current_yaw = 0.0
        
        # Transformer for converting between lat/lon and UTM
        self.transformer = None
        self.utm_zone = None
        
        # If using fixed starting point, initialize immediately
        if self.use_fixed_starting_point:
            self.initial_lat = self.fixed_latitude
            self.initial_lon = self.fixed_longitude
            self.initial_alt = 0.0  # Assuming altitude is 0 at fixed point
            self.gps_collection_complete = True
            self.setup_transformer()
            self.get_logger().info(f'Using fixed starting point: {self.initial_lat}, {self.initial_lon}')
        else:
            self.get_logger().info(f'Collecting GPS data for {self.initial_collection_time} seconds...')
        
        # Timer for regular publishing and debugging
        self.timer = self.create_timer(0.1, self.publish_filtered_gps)  # 10 Hz
        if self.debug_mode:
            self.debug_timer = self.create_timer(5.0, self.debug_info)  # Debug output every 5 seconds
        
        self.get_logger().info('GPS Filter Node initialized')
        
        if self.reverse_direction:
            self.get_logger().info('Movement direction is reversed')
    
    def debug_info(self):
        """Print debug information"""
        if hasattr(self, 'current_odom_x') and self.initial_odom_x is not None:
            dx = self.current_odom_x - self.initial_odom_x
            dy = self.current_odom_y - self.initial_odom_y
            direction = "reversed" if self.reverse_direction else "normal"
            self.get_logger().info(f"Direction: {direction}, Yaw: {math.degrees(self.current_yaw):.2f}°, " +
                                  f"Delta from odom: dx={dx:.3f}m, dy={dy:.3f}m")
        
    def gps_callback(self, msg):
        # Only used if not using fixed starting point
        if self.use_fixed_starting_point:
            return
            
        current_time = time.time()
        
        # Store GPS data during initial collection period
        if not self.gps_collection_complete:
            self.gps_data_buffer.append(msg)
            
            # Check if collection period is over
            if current_time - self.start_time >= self.initial_collection_time:
                self.process_initial_gps_data()
                self.gps_collection_complete = True
                self.get_logger().info('GPS data collection complete')
                self.get_logger().info(f'Initial position: {self.initial_lat}, {self.initial_lon}')
        
    def odom_callback(self, msg):
        # Extract orientation (yaw) from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        
        # Store initial odometry position when GPS collection is complete
        if self.gps_collection_complete and self.initial_odom_x is None:
            self.initial_odom_x = msg.pose.pose.position.x
            self.initial_odom_y = msg.pose.pose.position.y
            self.initial_yaw = yaw  # Store initial orientation
            self.get_logger().info(f'Initial odometry position: {self.initial_odom_x}, {self.initial_odom_y}')
            self.get_logger().info(f'Initial orientation (yaw): {math.degrees(self.initial_yaw):.2f} degrees')
        
        # Update current odometry position
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        
    def process_initial_gps_data(self):
        """Process collected GPS data to determine stable initial position"""
        
        # Filter out outliers using median absolute deviation
        latitudes = [msg.latitude for msg in self.gps_data_buffer]
        longitudes = [msg.longitude for msg in self.gps_data_buffer]
        altitudes = [msg.altitude for msg in self.gps_data_buffer]
        
        # Calculate median and MAD
        median_lat = statistics.median(latitudes)
        median_lon = statistics.median(longitudes)
        
        # Calculate absolute deviations
        lat_deviations = [abs(lat - median_lat) for lat in latitudes]
        lon_deviations = [abs(lon - median_lon) for lon in longitudes]
        
        # Calculate MAD (Median Absolute Deviation)
        mad_lat = statistics.median(lat_deviations)
        mad_lon = statistics.median(lon_deviations)
        
        # Use tighter filter for outliers
        filtered_lat_indices = [i for i, lat in enumerate(latitudes) if abs(lat - median_lat) <= 2 * mad_lat]
        filtered_lon_indices = [i for i, lon in enumerate(longitudes) if abs(lon - median_lon) <= 2 * mad_lon]
        
        # Get common indices that pass both filters
        common_indices = set(filtered_lat_indices).intersection(set(filtered_lon_indices))
        
        if len(common_indices) < 5:
            self.get_logger().warning(f'Not enough stable GPS readings (only {len(common_indices)}). Using median of all readings.')
            self.initial_lat = median_lat
            self.initial_lon = median_lon
            self.initial_alt = statistics.median(altitudes)
        else:
            # Use the mean of filtered readings
            filtered_latitudes = [latitudes[i] for i in common_indices]
            filtered_longitudes = [longitudes[i] for i in common_indices]
            filtered_altitudes = [altitudes[i] for i in common_indices]
            
            self.initial_lat = sum(filtered_latitudes) / len(filtered_latitudes)
            self.initial_lon = sum(filtered_longitudes) / len(filtered_longitudes)
            self.initial_alt = sum(filtered_altitudes) / len(filtered_altitudes)
            
            self.get_logger().info(f'Used {len(common_indices)}/{len(self.gps_data_buffer)} GPS readings for initialization')
        
        # Initialize transformer for lat/lon <-> UTM conversions
        self.setup_transformer()
        
    def setup_transformer(self):
        """Set up pyproj transformer for coordinate conversions"""
        # Determine UTM zone based on initial longitude
        self.utm_zone = int((self.initial_lon + 180) / 6) + 1
        utm_crs = f"+proj=utm +zone={self.utm_zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        
        # Create transformer
        self.transformer = Transformer.from_crs(
            "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs",
            utm_crs,
            always_xy=True
        )
        
        self.get_logger().info(f'Using UTM zone {self.utm_zone} for coordinate transformations')
    
    def publish_filtered_gps(self):
        """Publish filtered GPS data based on initial GPS and odometry changes"""
        if not self.gps_collection_complete or self.initial_odom_x is None:
            return
        
        # Create NavSatFix message
        filtered_gps = NavSatFix()
        filtered_gps.header.stamp = self.get_clock().now().to_msg()
        filtered_gps.header.frame_id = "base_link"
        
        # Calculate GPS position based on odometry
        if hasattr(self, 'current_odom_x') and hasattr(self, 'current_odom_y'):
            # Calculate displacement in meters in the odometry frame
            dx_odom = self.current_odom_x - self.initial_odom_x
            dy_odom = self.current_odom_y - self.initial_odom_y
            
            # Apply reverse direction if needed
            if self.reverse_direction:
                dx_odom = -dx_odom
                dy_odom = -dy_odom
            
            # Convert initial lat/lon to UTM
            initial_utm_x, initial_utm_y = self.transformer.transform(self.initial_lon, self.initial_lat)
            
            # Simplified approach: just apply the odometry deltas in world frame
            # This is simpler and avoids complex rotation calculations
            # Transform displacement from robot frame to world frame
            dx_world = dx_odom * math.cos(self.initial_yaw) - dy_odom * math.sin(self.initial_yaw)
            dy_world = dx_odom * math.sin(self.initial_yaw) + dy_odom * math.cos(self.initial_yaw)
            
            # Apply displacement to UTM coordinates
            current_utm_x = initial_utm_x + dx_world
            current_utm_y = initial_utm_y + dy_world
            
            # Convert back to lat/lon
            inv_transformer = Transformer.from_crs(
                f"+proj=utm +zone={self.utm_zone} +ellps=WGS84 +datum=WGS84 +units=m +no_defs",
                "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs",
                always_xy=True
            )
            current_lon, current_lat = inv_transformer.transform(current_utm_x, current_utm_y)
            
            # Populate message
            filtered_gps.latitude = current_lat
            filtered_gps.longitude = current_lon
            filtered_gps.altitude = self.initial_alt
            
            # Fixed covariance values for simplicity
            filtered_gps.position_covariance = [
                1.0, 0.0, 0.0,  # x variance and covariances
                0.0, 1.0, 0.0,  # y variance and covariances
                0.0, 0.0, 2.0   # z variance and covariances
            ]
            filtered_gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            # Publish
            self.filtered_gps_pub.publish(filtered_gps)

def main(args=None):
    rclpy.init(args=args)
    node = GPSFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()