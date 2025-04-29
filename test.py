#!/usr/bin/env python3
import socket
import time
import sys

# Configuration
TEENSY_IP = "192.168.1.177"  # Update this to match your Teensy's IP
TEENSY_PORT = 5002
MESSAGE = "Hello Teensy!"
BUFFER_SIZE = 1024


def send_udp_message(message, ip, port):
    """Send a UDP message and receive response"""
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5)  # 5 second timeout

    try:
        # Send message
        print(f"Sending to {ip}:{port}: {message}")
        sock.sendto(message.encode(), (ip, port))

        # Wait for response
        print("Waiting for response...")
        data, addr = sock.recvfrom(BUFFER_SIZE)
        print(f"Received from {addr[0]}:{addr[1]}: {data.decode()}")
        return True

    except socket.timeout:
        print("Error: Timeout waiting for response")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        sock.close()


def main():
    # Command line arguments handling
    ip = TEENSY_IP
    port = TEENSY_PORT
    message = MESSAGE

    if len(sys.argv) > 1:
        message = sys.argv[1]
    if len(sys.argv) > 2:
        ip = sys.argv[2]
    if len(sys.argv) > 3:
        port = int(sys.argv[3])

    # Send message
    send_udp_message(message, ip, port)


if __name__ == "__main__":
    main()
