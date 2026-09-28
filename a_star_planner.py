import heapq
import math
import numpy as np


def to_cell(p, res):
    return (int(p[0] / res), int(p[1] / res))          # (ix, iy)


def to_world(c, res):
    return ((c[0] + 0.5) * res, (c[1] + 0.5) * res)    # center of the cell


def heuristic(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)


def a_star(grid, start, goal, res):
    """Returns a list of world (x, y) points from start to goal, or None."""
    s, g = to_cell(start, res), to_cell(goal, res)
    h, w = grid.shape
    if not (0 <= g[0] < w and 0 <= g[1] < h) or grid[g[1], g[0]]:
        return None

    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]

    open_list = [(heuristic(s, g), 0.0, s)]
    came_from = {}
    cost = {s: 0.0}

    while open_list:
        _, c, cur = heapq.heappop(open_list)
        if cur == g:
            cells = [cur]
            while cur in came_from:
                cur = came_from[cur]
                cells.append(cur)
            cells.reverse()
            path = [to_world(c_, res) for c_ in cells]
            path[0], path[-1] = tuple(start), tuple(goal)
            return path
        if c > cost[cur]:
            continue
        for dx, dy, step in moves:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < w and 0 <= ny < h) or grid[ny, nx]:
                continue
            # don't cut corners diagonally
            if dx != 0 and dy != 0 and (grid[cur[1], nx] or grid[ny, cur[0]]):
                continue
            new_cost = c + step
            if new_cost < cost.get((nx, ny), float("inf")):
                cost[(nx, ny)] = new_cost
                came_from[(nx, ny)] = cur
                heapq.heappush(open_list, (new_cost + heuristic((nx, ny), g), new_cost, (nx, ny)))
    return None


def line_is_free(grid, a, b, res):
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = int(dist / (res / 2)) + 1
    for i in range(n + 1):
        t = i / n
        ix, iy = to_cell((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), res)
        if grid[iy, ix]:
            return False
    return True


def smooth_path(path, grid, res):
    """Shortcutting: skip ahead to the farthest waypoint we can see in a straight line."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_is_free(grid, path[i], path[j], res):
            j -= 1
        out.append(path[j])
        i = j
    return out


def densify(path, step=0.3):
    """Add evenly spaced points along each segment so the controller has something to chase."""
    out = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(dist / step))
        for i in range(1, n + 1):
            t = i / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def plan_path(grid, start, goal, res):
    """A* + smoothing. Returns dense waypoints or None if there's no route."""
    raw = a_star(grid, start, goal, res)
    if raw is None:
        return None
    return densify(smooth_path(raw, grid, res))
