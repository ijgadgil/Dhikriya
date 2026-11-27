"""Helpers shared by the ROS 2 nodes (imports rclpy lazily; not used by the unit tests)."""
import json
import math
import os
import threading
import time

from .geometry import odom_to_world, yaw_from_quat
from .paths import run_dir, scenario_path, town_path
from .scenario import Scenario
from .world import World


def load_scenario(node):
    node.declare_parameter('scenario', 's0_straight_control')
    sid = node.get_parameter('scenario').value
    world = World.from_yaml(town_path())
    return world, Scenario.from_yaml(scenario_path(sid), world)


class JsonLog:
    """Thread-safe JSON-lines log in the run directory; every line carries wall-clock time t."""

    def __init__(self, name):
        self.path = os.path.join(run_dir(), name + '.jsonl')
        self.f = open(self.path, 'a', buffering=1)
        self.lock = threading.Lock()

    def write(self, **kw):
        kw.setdefault('t', time.time())
        with self.lock:
            self.f.write(json.dumps(kw, default=str) + '\n')


class PoseTracker:
    """World pose, speed and zone of the ego vehicle from /vla/odom (odometry starts at the spawn pose)."""

    def __init__(self, node, world, spawn, topic='/vla/odom'):
        from nav_msgs.msg import Odometry
        self.world, self.spawn = world, tuple(spawn)
        self.x = self.y = self.yaw = self.v = None
        self.zone, self.s_in, self.stamp = None, 0.0, None
        node.declare_parameter('odom_is_world', False)
        self.is_world = node.get_parameter('odom_is_world').value
        node.create_subscription(Odometry, topic, self._cb, 10)

    def _cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.is_world:
            self.x, self.y, self.yaw = p.x, p.y, yaw
        else:
            self.x, self.y, self.yaw = odom_to_world(self.spawn, p.x, p.y, yaw)
        self.v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)   # speed, whatever the twist frame
        self.zone, self.s_in, _ = self.world.locate(self.x, self.y)
        self.stamp = time.time()

    @property
    def ready(self):
        return self.stamp is not None


def redis_store(node):
    import redis
    from .context import RedisContextStore
    node.declare_parameter('redis_host', 'localhost')
    node.declare_parameter('redis_port', 6379)
    r = redis.Redis(host=node.get_parameter('redis_host').value, port=node.get_parameter('redis_port').value)
    return RedisContextStore(r), r
