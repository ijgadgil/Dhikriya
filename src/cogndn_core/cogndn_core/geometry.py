"""Pose helpers shared by the ROS nodes (no ROS imports)."""
import math


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odom_to_world(spawn, ox, oy, oyaw):
    """Ackermann odometry starts at the spawn pose; convert it to world coordinates.
    spawn = (x, y, yaw) of the vehicle in the world when the simulation started."""
    sx, sy, syaw = spawn
    c, s = math.cos(syaw), math.sin(syaw)
    return sx + c * ox - s * oy, sy + s * ox + c * oy, syaw + oyaw
