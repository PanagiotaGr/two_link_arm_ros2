#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # ΝΕΟ

class TwoLinkIK(Node):
    def __init__(self):
        super().__init__('two_link_ik')
        self.L1 = 0.3  # μήκος 1ου συνδέσμου (m)
        self.L2 = 0.2  # μήκος 2ου συνδέσμου (m)

        # τρέχων στόχος (x, y)
        self.target_x = 0.3
        self.target_y = 0.1

        # subscriber σε στόχο
        self.target_sub = self.create_subscription(
            Point,
            'target_point',       # όνομα topic
            self.target_callback, # callback συνάρτηση
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('TwoLinkIK node started')

    def target_callback(self, msg: Point):
        """Δέχεται νέο στόχο από topic."""
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info(
            f'New target received: x={self.target_x:.2f}, y={self.target_y:.2f}'
        )

    def ik_2link(self, x, y):
        r2 = x**2 + y**2
        c2 = (r2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        if c2 < -1.0 or c2 > 1.0:
            raise ValueError("Στόχος εκτός reach")

        s2 = math.sqrt(1 - c2**2)  # elbow-up
        theta2 = math.atan2(s2, c2)

        k1 = self.L1 + self.L2 * c2
        k2 = self.L2 * s2
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return theta1, theta2

    def timer_callback(self):
        x, y = self.target_x, self.target_y
        try:
            th1, th2 = self.ik_2link(x, y)
            self.get_logger().info(
                f"Target ({x:.2f}, {y:.2f}) -> θ1={th1:.2f} rad, θ2={th2:.2f} rad"
            )
        except ValueError as e:
            self.get_logger().warn(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = TwoLinkIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

