#!/usr/bin/env python3
"""Drive a known collision-free route to expose each room to the simulated LiDAR."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


WAYPOINTS = [
    (-4.0, 0.0), (-4.0, 3.0), (-4.0, 0.0), (-4.0, -2.7),
    (-4.0, 0.0), (-1.2, 0.7), (1.3, 0.7), (3.0, 0.0),
    (3.4, 2.8), (3.0, 0.0), (4.0, 0.0), (4.0, -3.0),
    (4.0, 0.0), (4.8, 0.0),
]


class MappingDriver(Node):
    def __init__(self):
        super().__init__('room_mapping_driver')
        self.pose = None
        self.index = 0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def tick(self):
        if self.pose is None:
            return
        command = Twist()
        if self.index == len(WAYPOINTS):
            self.publisher.publish(command)
            return
        x, y, yaw = self.pose
        goal_x, goal_y = WAYPOINTS[self.index]
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        if distance < 0.12:
            self.index += 1
            self.get_logger().info(f'Waypoint {self.index}/{len(WAYPOINTS)} reached')
        else:
            speed = min(0.32, distance * 0.8)
            command.linear.x = speed * (math.cos(yaw) * dx + math.sin(yaw) * dy) / distance
            command.linear.y = speed * (-math.sin(yaw) * dx + math.cos(yaw) * dy) / distance
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = MappingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
