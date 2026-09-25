import math

from nav_msgs.msg import OccupancyGrid

from room_nav.navigator import plan
from room_nav.world import GOAL, HEIGHT, RESOLUTION, START, WIDTH, occupied, raycast


def room_grid():
    grid = OccupancyGrid()
    grid.info.resolution = RESOLUTION
    grid.info.width = int(WIDTH / RESOLUTION)
    grid.info.height = int(HEIGHT / RESOLUTION)
    grid.info.origin.position.x = -WIDTH / 2
    grid.info.origin.position.y = -HEIGHT / 2
    grid.data = [100 if occupied(-WIDTH / 2 + (i + .5) * RESOLUTION,
                                 -HEIGHT / 2 + (j + .5) * RESOLUTION) else 0
                 for j in range(grid.info.height) for i in range(grid.info.width)]
    return grid


def test_laser_hits_block_and_wall():
    assert math.isclose(raycast(*START, 0), 2.9)
    assert math.isclose(raycast(*START, math.pi), 0.6)


def test_planner_routes_around_central_block_with_clearance():
    route = plan(room_grid(), START, GOAL)
    assert route
    assert any(abs(x) < .35 and abs(y) > .9 for x, y in route)
    assert all(not occupied(x, y, .29) for x, y in route)
    assert math.dist(route[-1], GOAL) < .1
