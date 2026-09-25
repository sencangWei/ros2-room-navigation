"""Plan around the room's obstacles, then follow the path with laser safety."""

import heapq
import math
from functools import lru_cache

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .world import GOAL, ROBOT_RADIUS


def plan(grid, start, goal, clearance=ROBOT_RADIUS + 0.20):
    """A* on an inflated OccupancyGrid; return world-coordinate cell centres."""
    info = grid.info
    width, height, resolution = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y

    def cell(point):
        return (int((point[0] - ox) / resolution), int((point[1] - oy) / resolution))

    @lru_cache(maxsize=None)
    def blocked(i, j):
        if not (0 <= i < width and 0 <= j < height):
            return True
        radius = math.ceil(clearance / resolution)
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if math.hypot(di, dj) * resolution > clearance:
                    continue
                x, y = i + di, j + dj
                if not (0 <= x < width and 0 <= y < height):
                    return True
                if grid.data[y * width + x] != 0:
                    return True
        return False

    source, target = cell(start), cell(goal)
    if blocked(*source) or blocked(*target):
        return []
    frontier = [(0.0, source)]
    cost = {source: 0.0}
    previous = {}
    directions = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == target:
            cells = [current]
            while current != source:
                current = previous[current]
                cells.append(current)
            return [(ox + (i + 0.5) * resolution, oy + (j + 0.5) * resolution)
                    for i, j in reversed(cells)]
        for di, dj in directions:
            neighbor = (current[0] + di, current[1] + dj)
            if blocked(*neighbor) or (di and dj and
                                      (blocked(current[0] + di, current[1]) or
                                       blocked(current[0], current[1] + dj))):
                continue
            new_cost = cost[current] + math.hypot(di, dj)
            if new_cost < cost.get(neighbor, math.inf):
                cost[neighbor] = new_cost
                previous[neighbor] = current
                priority = new_cost + math.hypot(target[0] - neighbor[0],
                                                  target[1] - neighbor[1])
                heapq.heappush(frontier, (priority, neighbor))
    return []


class Navigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        self.grid = None
        self.pose = None
        self.scan = None
        self.path = []
        self.path_index = 0
        self.done = False
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', map_qos)
        self.create_timer(0.1, self.tick)

    def on_map(self, msg):
        self.grid = msg

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_scan(self, msg):
        self.scan = msg

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def tick(self):
        if self.done or self.grid is None or self.pose is None or self.scan is None:
            return
        x, y, yaw = self.pose
        if not self.path:
            self.path = plan(self.grid, (x, y), GOAL)
            if not self.path:
                self.get_logger().error('No collision-free path to goal.')
                self.done = True
                return
            self.publish_path()
            self.get_logger().info(f'Planned {len(self.path)} cells around the obstacles.')

        command = Twist()
        if math.hypot(GOAL[0] - x, GOAL[1] - y) < 0.18:
            self.done = True
            self.command_pub.publish(command)
            self.get_logger().info('REACHED_GOAL: robot arrived at the opposite side of the room.')
            return

        while (self.path_index + 1 < len(self.path) and
               math.hypot(self.path[self.path_index][0] - x,
                          self.path[self.path_index][1] - y) < 0.25):
            self.path_index += 1
        target_index = self.path_index
        while (target_index + 1 < len(self.path) and
               math.hypot(self.path[target_index][0] - x,
                          self.path[target_index][1] - y) < 0.32):
            target_index += 1
        target_x, target_y = self.path[target_index]
        error = math.atan2(target_y - y, target_x - x) - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        command.angular.z = max(-1.2, min(1.2, 2.5 * error))
        command.linear.x = 0.32 * max(0.0, 1.0 - abs(error) / 1.0)

        # A fresh scan gates forward motion if the path gets too close to a wall.
        front = [distance for k, distance in enumerate(self.scan.ranges)
                 if abs(self.scan.angle_min + k * self.scan.angle_increment) < math.pi / 9
                 and math.isfinite(distance)]
        if front and min(front) < 0.28:
            command.linear.x = 0.0
        self.command_pub.publish(command)


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
