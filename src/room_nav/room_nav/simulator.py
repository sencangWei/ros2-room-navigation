"""A small deterministic 2-D differential-drive robot and laser simulator."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .world import GOAL, HEIGHT, RESOLUTION, ROBOT_RADIUS, START, WIDTH, occupied, raycast


def quaternion(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class Simulator(Node):
    def __init__(self):
        super().__init__('room_simulator')
        self.x, self.y = START
        self.yaw = 0.0
        self.linear = 0.0
        self.angular = 0.0
        self.last_command = self.get_clock().now()
        self.last_tick = self.last_command
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/room_markers', map_qos)
        self.tf = TransformBroadcaster(self)
        self.publish_map()
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Room ready. Robot starts at (-3.25, 0); goal is (3.25, 0).')

    def on_command(self, msg):
        self.linear = max(-0.2, min(0.4, msg.linear.x))
        self.angular = max(-1.5, min(1.5, msg.angular.z))
        self.last_command = self.get_clock().now()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RESOLUTION
        msg.info.width = int(WIDTH / RESOLUTION)
        msg.info.height = int(HEIGHT / RESOLUTION)
        msg.info.origin.position.x = -WIDTH / 2
        msg.info.origin.position.y = -HEIGHT / 2
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if occupied(-WIDTH / 2 + (i + 0.5) * RESOLUTION,
                                    -HEIGHT / 2 + (j + 0.5) * RESOLUTION) else 0
                    for j in range(msg.info.height) for i in range(msg.info.width)]
        self.map_pub.publish(msg)

    def publish_markers(self):
        markers = MarkerArray()
        for index, (x, y, sx, sy, color) in enumerate(
            [(GOAL[0], GOAL[1], 0.35, 0.35, (0.1, 0.9, 0.1)),
             (START[0], START[1], 0.25, 0.25, (0.1, 0.4, 0.9))]
        ):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.ns = 'landmarks'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = x, y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, 0.1
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            markers.markers.append(marker)
        robot = Marker()
        robot.header.frame_id = 'odom'
        robot.header.stamp = self.get_clock().now().to_msg()
        robot.ns = 'robot'
        robot.id = 0
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose.position.x, robot.pose.position.y = self.x, self.y
        robot.pose.position.z = 0.13
        robot.pose.orientation.z, robot.pose.orientation.w = quaternion(self.yaw)
        robot.scale.x, robot.scale.y, robot.scale.z = 0.4, 0.22, 0.13
        robot.color.r, robot.color.g, robot.color.b, robot.color.a = 0.1, 0.45, 1.0, 1.0
        markers.markers.append(robot)
        self.marker_pub.publish(markers)

    def tick(self):
        now = self.get_clock().now()
        dt = min(0.1, (now - self.last_tick).nanoseconds / 1e9)
        self.last_tick = now
        if (now - self.last_command).nanoseconds / 1e9 > 0.5:
            self.linear = self.angular = 0.0
        new_yaw = self.yaw + self.angular * dt
        new_x = self.x + self.linear * math.cos(new_yaw) * dt
        new_y = self.y + self.linear * math.sin(new_yaw) * dt
        if not occupied(new_x, new_y, ROBOT_RADIUS):
            self.x, self.y = new_x, new_y
        self.yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
        stamp = now.to_msg()
        qz, qw = quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = self.linear
        odom.twist.twist.angular.z = self.angular
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z, transform.transform.rotation.w = qz, qw
        self.tf.sendTransform(transform)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 36  # 72 rays, every 5 degrees
        scan.angle_max = math.pi - scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 5.0
        scan.ranges = [raycast(self.x, self.y, self.yaw + scan.angle_min + k * scan.angle_increment)
                       for k in range(72)]
        self.scan_pub.publish(scan)
        self.publish_markers()


def main():
    rclpy.init()
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
