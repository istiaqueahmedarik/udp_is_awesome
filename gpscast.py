import socket
import struct
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

CURRENT_LAT = 23.83676242944313
CURRENT_LON = 90.36332887435273

UDP_IP = "239.0.0.16"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


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


if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
    print("Usage: gpscast.py [tx|rx]")
    sys.exit(1)

if sys.argv[1] == "tx":
    print(f"Current GPS position: {CURRENT_LAT}, {CURRENT_LON}")
    print("Enter target GPS coordinates. Type 'quit' to exit.")

    target_points = []

    while True:
        coord_input = input(
            'Enter GPS coordinates (latitude,longitude) or "quit": ')
        if coord_input.lower() == 'quit':
            break

        try:
            lat, lon = map(float, coord_input.split(','))
            target_points.append((lat, lon))

            bearing = calculate_bearing(CURRENT_LAT, CURRENT_LON, lat, lon)

            shortest_bearing = bearing
            if shortest_bearing > 180:
                shortest_bearing = -(360 - shortest_bearing)
            if shortest_bearing < -180:
                shortest_bearing = 360 + shortest_bearing
            print(f"Shortest bearing: {shortest_bearing:.1f}°")

            distance = calculate_distance(CURRENT_LAT, CURRENT_LON, lat, lon)

            print(f"Bearing: {bearing:.1f}° | Distance: {distance:.1f} meters")

            motor_value = shortest_bearing * (7.0/90.0)

            motor_num = input('Enter motor number (1-4): ')
            combined = f"{motor_num} {motor_value}"
            sock.sendto(combined.encode('utf-8'), (UDP_IP, UDP_PORT))

        except ValueError:
            print("Invalid input. Please enter coordinates in format: latitude,longitude")
        except Exception as e:
            print(f"Error: {e}")

else:
    sock.bind((UDP_IP, UDP_PORT))
    print("Receiving GPS angles:")
    rclpy.init()

    class AngleReceiver(Node):
        def __init__(self):
            super().__init__('angle_receiver')
            qos_profile = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT
            )
            self.publisher_ = self.create_publisher(
                String, 'position', qos_profile)

        def publish_angle(self, data):
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)

    receiver = AngleReceiver()

    try:
        while rclpy.ok():
            data, addr = sock.recvfrom(1024)
            if data:
                angle_data = data.decode('utf-8')
                receiver.publish_angle(angle_data)
                print(f"Received: {angle_data}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()
