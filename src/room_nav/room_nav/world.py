"""Geometry shared by the map, simulated laser, and collision check (metres)."""

import math

WIDTH = 8.0
HEIGHT = 5.0
RESOLUTION = 0.1
ROBOT_RADIUS = 0.18
START = (-3.25, 0.0)
GOAL = (3.25, 0.0)

# x_min, y_min, x_max, y_max; the central block forces a detour.
OBSTACLES = ((-0.35, -0.9, 0.35, 0.9), (1.5, -2.0, 1.9, -0.8))


def occupied(x, y, radius=0.0):
    if abs(x) + radius >= WIDTH / 2 - 0.15 or abs(y) + radius >= HEIGHT / 2 - 0.15:
        return True
    return any(
        left - radius <= x <= right + radius
        and bottom - radius <= y <= top + radius
        for left, bottom, right, top in OBSTACLES
    )


def raycast(x, y, angle, max_range=5.0):
    """Return distance to the nearest axis-aligned wall or block."""
    dx, dy = math.cos(angle), math.sin(angle)
    boxes = ((-WIDTH / 2 + 0.15, -HEIGHT / 2 + 0.15,
              WIDTH / 2 - 0.15, HEIGHT / 2 - 0.15),) + OBSTACLES
    nearest = max_range
    for left, bottom, right, top in boxes:
        for boundary, direction, low, high, origin, other_origin, other_direction in (
            (left, dx, bottom, top, x, y, dy),
            (right, dx, bottom, top, x, y, dy),
            (bottom, dy, left, right, y, x, dx),
            (top, dy, left, right, y, x, dx),
        ):
            if abs(direction) < 1e-9:
                continue
            distance = (boundary - origin) / direction
            other = other_origin + distance * other_direction
            if 0.0 <= distance <= nearest and low <= other <= high:
                nearest = distance
    return nearest
