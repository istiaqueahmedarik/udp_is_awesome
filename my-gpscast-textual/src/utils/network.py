import socket
import struct

class Network:
    def __init__(self, udp_ip="239.0.0.16", udp_port=1234):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = self.setup_socket()

    def setup_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        mreq = struct.pack("4sl", socket.inet_aton(self.udp_ip), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return sock

    def send_data(self, data):
        self.sock.sendto(data.encode('utf-8'), (self.udp_ip, self.udp_port))

    def receive_data(self):
        data, addr = self.sock.recvfrom(1024)
        return data.decode('utf-8'), addr