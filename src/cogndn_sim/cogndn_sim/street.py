"""Pedestrian scripts shared by the world generator (animated actors) and street_bodies_node (physics bodies).

Each pedestrian walks its two-point path back and forth, turning for 0.6 s (plus an optional pause) at each end.
The same waypoint list drives both the visible animated actor and its invisible collision body, so the two
stay together; the body is what the car physically collides with.
"""
import math
import os

import yaml

TURN_S = 0.6


def load(cfg_dir):
    return yaml.safe_load(open(os.path.join(cfg_dir, 'street_indian.yaml')))


def active(st, scenario, group):
    """True if `group` (pedestrians, cows, ...) is used in `scenario`."""
    return group not in set(st.get('exclude', {}).get(scenario, []))


def waypoints(p):
    """[(t, x, y, yaw)] for one loop of pedestrian `p`; the last point equals the first (loop closed)."""
    (x0, y0), (x1, y1) = p['path']
    v, pause = p['speed'], p.get('pause_s', 0.0)
    L, head = math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0)
    t, wp = 0.0, []
    for (ax, ay), (bx, by), yaw in (((x0, y0), (x1, y1), head), ((x1, y1), (x0, y0), head + math.pi)):
        wp.append((t, ax, ay, yaw))
        t += L / v
        wp.append((t, bx, by, yaw))
        t += TURN_S + pause
    wp.append((t, x0, y0, head))
    return wp


def state_at(wp, t):
    """Position (x, y) and velocity (vx, vy) of a looped waypoint script at time t (seconds)."""
    period = wp[-1][0]
    t = t % period
    for (ta, xa, ya, _), (tb, xb, yb, _) in zip(wp, wp[1:]):
        if ta <= t <= tb:
            if tb - ta < 1e-9:
                return (xb, yb), (0.0, 0.0)
            u = (t - ta) / (tb - ta)
            return (xa + u * (xb - xa), ya + u * (yb - ya)), ((xb - xa) / (tb - ta), (yb - ya) / (tb - ta))
    return (wp[0][1], wp[0][2]), (0.0, 0.0)
