#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import time
from mavros_msgs.msg import RCIn, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode


class RCOverrideTest(Node):
    def __init__(self):
        super().__init__('rc_override_test')

        # Publisher for RC override
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn, '/mavros/rc/override', 10)

        # RC channel ranges (typical PWM values)
        self.rc_min = 1000
        self.rc_max = 2000
        self.rc_mid = 1500

        # Create timer for publishing at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_random_rc)

        self.get_logger().info("RC Override Test Node Started")

    def generate_random_rc_values(self):
        """Generate random RC values for testing"""
        rc_override = OverrideRCIn()

        # Generate random values for 8 channels
        rc_override.channels = [
            random.randint(self.rc_min, self.rc_max),  # Channel 1 (Roll)
            random.randint(self.rc_min, self.rc_max),  # Channel 2 (Pitch)
            random.randint(self.rc_min, self.rc_max),  # Channel 3 (Throttle)
            random.randint(self.rc_min, self.rc_max),  # Channel 4 (Yaw)
            random.randint(self.rc_min, self.rc_max),  # Channel 5
            random.randint(self.rc_min, self.rc_max),  # Channel 6
            random.randint(self.rc_min, self.rc_max),  # Channel 7
            random.randint(self.rc_min, self.rc_max)   # Channel 8
        ]

        return rc_override

    def publish_random_rc(self):
        """Timer callback to publish random RC values"""
        rc_override = self.generate_random_rc_values()
        self.rc_override_pub.publish(rc_override)

        self.get_logger().info(
            f"Published RC Override: {rc_override.channels}")

    def stop_override(self):
        """Stop RC override by publishing neutral values"""
        rc_override = OverrideRCIn()
        rc_override.channels = [0] * 8  # 0 disables override
        self.rc_override_pub.publish(rc_override)
        self.get_logger().info("RC Override stopped")


def main(args=None):
    rclpy.init(args=args)

    try:
        rc_test = RCOverrideTest()
        rclpy.spin(rc_test)
    except KeyboardInterrupt:
        pass
    finally:
        if 'rc_test' in locals():
            rc_test.stop_override()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
