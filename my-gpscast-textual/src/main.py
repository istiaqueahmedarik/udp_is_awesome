from textual.app import App
from textual.containers import Container  # Updated import
from textual.widgets import Header, Footer, Static, Input, Button
from textual.reactive import Reactive
from textual import events
import asyncio
import struct
import socket
import math
import sys
from gpscast import calculate_bearing, calculate_distance, CURRENT_LAT, CURRENT_LON
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

UDP_IP = "239.0.0.16"
UDP_PORT = 1234


class GPSCastApp(App):
    target_lat: Reactive[float] = Reactive(0.0)
    target_lon: Reactive[float] = Reactive(0.0)
    bearing: Reactive[float] = Reactive(0.0)
    distance: Reactive[float] = Reactive(0.0)
    rx_task = None  # to hold the UDP receiver task
    angle_receiver = None  # new ROS node for publishing

    def compose(self):
        yield Header()
        yield Container(
            Button("TX Mode", id="tx_mode_button"),
            Button("RX Mode", id="rx_mode_button"),
            Button("Help", id="help_button"),  # new help button
            id="mode_select"
        )
        yield Container(
            Static(
                "Welcome! Please select a mode using the buttons above.", classes="intro"),
            id="content"
        )
        yield Footer()

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        # Mode selection and Help
        if event.button.id == "tx_mode_button":
            if self.rx_task:
                self.rx_task.cancel()
                self.rx_task = None
            self.show_tx_ui()
        elif event.button.id == "rx_mode_button":
            self.show_rx_ui()
            if not self.angle_receiver:
                rclpy.init(args=None)  # initialize ROS if not already
                self.angle_receiver = AngleReceiver()
            self.rx_task = asyncio.create_task(self.listen_udp())
        elif event.button.id == "help_button":
            self.show_help()
        # TX calculate & send button
        elif event.button.id == "calculate_send_button":
            coord_input = self.query_one("#coord_input").value
            motor_input = self.query_one("#motor_input").value
            tx_output = self.query_one("#tx_output")
            try:
                lat, lon = map(float, coord_input.split(','))
                self.target_lat = lat
                self.target_lon = lon

                self.bearing = calculate_bearing(
                    CURRENT_LAT, CURRENT_LON, lat, lon)
                self.distance = calculate_distance(
                    CURRENT_LAT, CURRENT_LON, lat, lon)

                # calculate shortest bearing
                shortest_bearing = self.bearing
                if shortest_bearing > 180:
                    shortest_bearing = -(360 - shortest_bearing)
                if shortest_bearing < -180:
                    shortest_bearing = 360 + shortest_bearing

                motor_value = shortest_bearing * (7.0 / 90.0)
                combined = f"{motor_input} {motor_value}"
                self.send_udp_data(combined)
                tx_output.update(f"[b green]TX Mode Active[/b green]\n"
                                 f"Bearing: {self.bearing:.1f}° | Distance: {self.distance:.1f}m\n"
                                 f"Sent: {combined}")
            except ValueError:
                tx_output.update(
                    "[b red]Invalid input. Use: latitude,longitude[/b red]")

    def show_help(self):
        content = self.query_one("#content")
        for child in list(content.children):
            child.remove()
        # Display a detailed explanation
        help_text = (
            "[b yellow]GPSCast Textual Application Help[/b yellow]\n\n"
            "[b]TX Mode:[/b] Calculate bearing and distance from your current position "
            "to a target coordinate. You can also specify a motor number to send a custom command.\n\n"
            "[b]RX Mode:[/b] Listen for incoming UDP messages and publish them over ROS. "
            "This mode displays live updates of received angle data.\n\n"
            "Use the buttons above to switch modes. Enjoy the insane UI!"
        )
        content.mount(Static(help_text, classes="help_text"))

    def show_tx_ui(self):
        content = self.query_one("#content")
        for child in list(content.children):
            child.remove()
        content.mount(
            Input(placeholder="Enter target coordinates (lat,lon)",
                  id="coord_input"),
            Input(placeholder="Enter motor number", id="motor_input"),
            Button("Calculate & Send", id="calculate_send_button"),
            Static(id="tx_output", classes="tx_output")
        )

    def show_rx_ui(self):
        content = self.query_one("#content")
        for child in list(content.children):
            child.remove()
        content.mount(
            Static("Waiting for UDP messages...",
                   id="rx_output", classes="rx_output")
        )

    def send_udp_data(self, data: str):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(data.encode('utf-8'), (UDP_IP, UDP_PORT))

    async def listen_udp(self):
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        mreq = struct.pack("4sl", socket.inet_aton(UDP_IP), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.bind((UDP_IP, UDP_PORT))
        sock.setblocking(True)
        rx_output = self.query_one("#rx_output")
        try:
            while True:
                data, addr = await loop.run_in_executor(None, sock.recvfrom, 1024)
                if data:
                    angle_data = data.decode('utf-8')
                    self.call_later(rx_output.update,
                                    f"Received: {angle_data}")
                    # Publish data via ROS
                    if self.angle_receiver:
                        self.angle_receiver.publish_angle(angle_data)
        except asyncio.CancelledError:
            sock.close()


# New class: ROS publisher node for angle messages
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

    def publish_angle(self, data: str):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)


if __name__ == "__main__":
    GPSCastApp().run()
    # Optionally, shutdown ROS if needed
    if rclpy.ok():
        rclpy.shutdown()
