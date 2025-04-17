import socket
import struct
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

import matplotlib.pyplot as plt

# Current position will be updated from MAVROS
CURRENT_LAT = 23.83678

CURRENT_LON = 90.36336


UDP_IP = "239.0.0.16"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

plt.ion()
fig, ax = plt.subplots(figsize=(10, 8))


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlon = lon2 - lon1

    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * \
        math.cos(lat2) * math.cos(dlon)

    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360

    return bearing


def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)

    a = math.sin(delta_lat/2) * math.sin(delta_lat/2) + \
        math.cos(lat1_rad) * math.cos(lat2_rad) * \
        math.sin(delta_lon/2) * math.sin(delta_lon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c


def update_plot(current_lat, current_lon, target_points):
    ax.clear()

    x_vals = []
    y_vals = []
    labels = []

    for i, (lat, lon) in enumerate(target_points):
        ns_distance = calculate_distance(
            current_lat, current_lon, lat, current_lon)
        if lat < current_lat:
            ns_distance = -ns_distance

        ew_distance = calculate_distance(
            current_lat, current_lon, current_lat, lon)
        if lon < current_lon:
            ew_distance = -ew_distance

        x_vals.append(ew_distance)
        y_vals.append(ns_distance)
        labels.append(f"Point {i+1}")

        distance = calculate_distance(current_lat, current_lon, lat, lon)
        ax.annotate(f"{distance:.1f}m", (ew_distance, ns_distance),
                    textcoords="offset points", xytext=(0, 10), ha='center')

    ax.scatter(0, 0, c='red', s=100, label='Current Position')

    ax.scatter(x_vals, y_vals, c='blue', s=50)

    for i, label in enumerate(labels):
        ax.annotate(label, (x_vals[i], y_vals[i]),
                    textcoords="offset points", xytext=(0, -15), ha='center')

    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_xlabel('East-West distance (meters)')
    ax.set_ylabel('North-South distance (meters)')
    ax.set_title('GPS Coordinates Relative to Current Position')
    ax.legend()

    fig.canvas.draw()
    plt.pause(0.1)
    print("Plot updated dynamically")


class GpsVizNode(Node):
    def __init__(self):
        super().__init__('gps_viz_node')

        # Create QoS profile compatible with MAVROS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to MAVROS GPS topic with appropriate QoS
        self.subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile)

        self.current_lat = CURRENT_LAT
        self.current_lon = CURRENT_LON
        self.target_points = []

        # Hard-coded target points (add your own if needed)
        # Comment out or modify these lines as needed
        # Example point ~100m away
        self.target_points.append((23.83678, 90.36336
                                   ))

        self.get_logger().info(
            f"Starting GPS visualization. Current position will be updated from MAVROS.")
        self.get_logger().info(
            f"Initial position: {self.current_lat}, {self.current_lon}")

    def gps_callback(self, msg):
        # Update current position from MAVROS
        self.current_lat = round(msg.latitude, 5)
        self.current_lon = round(msg.longitude, 5)

        self.get_logger().info(
            f"GPS updated: {self.current_lat}, {self.current_lon}")

        # Process each target point
        for lat, lon in self.target_points:
            # Calculate bearing and distance
            bearing = calculate_bearing(
                self.current_lat, self.current_lon, lat, lon)

            shortest_bearing = bearing
            if shortest_bearing > 180:
                shortest_bearing = -(360 - shortest_bearing)
            if shortest_bearing < -180:
                shortest_bearing = 360 + shortest_bearing

            distance = calculate_distance(
                self.current_lat, self.current_lon, lat, lon)

            self.get_logger().info(f"Target: {lat}, {lon}")
            self.get_logger().info(
                f"Bearing: {bearing:.1f}° | Shortest: {shortest_bearing:.1f}° | Distance: {distance:.1f} meters")

            # Calculate motor value and send over UDP
            motor_value = shortest_bearing * (7.0/90.0)

            # Use fixed motor number 0
            motor_num = 0
            combined = f"{motor_num} {motor_value}"
            sock.sendto(combined.encode('utf-8'), (UDP_IP, UDP_PORT))

        # Update the visualization
        update_plot(self.current_lat, self.current_lon, self.target_points)


def main(args=None):
    rclpy.init(args=args)

    gps_viz_node = GpsVizNode()

    try:
        rclpy.spin(gps_viz_node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        gps_viz_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
