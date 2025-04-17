import socket
import struct
import sys
import math
from utils.calculations import calculate_bearing, calculate_distance

CURRENT_LAT = 23.83676242944313
CURRENT_LON = 90.36332887435273

UDP_IP = "239.0.0.16"
UDP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

def handle_tx_mode():
    print(f"Current GPS position: {CURRENT_LAT}, {CURRENT_LON}")
    print("Enter target GPS coordinates. Type 'quit' to exit.")

    target_points = []

    while True:
        coord_input = input('Enter GPS coordinates (latitude,longitude) or "quit": ')
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

            motor_value = shortest_bearing * (7.0 / 90.0)
            motor_num = input('Enter motor number (1-4): ')
            combined = f"{motor_num} {motor_value}"
            sock.sendto(combined.encode('utf-8'), (UDP_IP, UDP_PORT))

        except ValueError:
            print("Invalid input. Please enter coordinates in format: latitude,longitude")
        except Exception as e:
            print(f"Error: {e}")

def handle_rx_mode():
    sock.bind((UDP_IP, UDP_PORT))
    print("Receiving GPS angles:")
    while True:
        data, addr = sock.recvfrom(1024)
        if data:
            angle_data = data.decode('utf-8')
            print(f"Received: {angle_data}", flush=True)

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ["tx", "rx"]:
        print("Usage: gpscast.py [tx|rx]")
        sys.exit(1)

    if sys.argv[1] == "tx":
        handle_tx_mode()
    else:
        handle_rx_mode()