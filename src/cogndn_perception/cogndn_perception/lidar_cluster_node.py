"""Bins the LiDAR returns ahead of the car into obstacle clusters -> /cogndn/obstacles."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from cogndn_core.pointcloud import parse_cloud
from cogndn_msgs.msg import Obstacle, ObstacleArray


def clusters(xyz, ground_z=-1.8, min_h=0.25, r_min=3.0, r_max=30.0, fov_deg=120.0, bin_deg=3.0, merge_m=1.5):
    if len(xyz) == 0:
        return []
    h = xyz[:, 2] - ground_z
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    b = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    m = (h >= min_h) & (r >= r_min) & (r <= r_max) & (np.abs(b) <= fov_deg / 2)
    r, b = r[m], b[m]
    nb = int(fov_deg / bin_deg)
    idx = np.clip(((b + fov_deg / 2) / bin_deg).astype(int), 0, nb - 1)
    best = np.full(nb, np.inf)
    cnt = np.zeros(nb, int)
    np.minimum.at(best, idx, r)
    np.add.at(cnt, idx, 1)
    out, cur = [], None
    for i in range(nb):
        if not math.isfinite(best[i]) or cnt[i] < 2:
            cur = None
            continue
        ang = -fov_deg / 2 + (i + 0.5) * bin_deg
        if cur and abs(best[i] - cur['range']) < merge_m:
            cur['bins'].append(ang)
            cur['range'] = min(cur['range'], best[i])
            cur['points'] += int(cnt[i])
        else:
            cur = {'range': float(best[i]), 'bins': [ang], 'points': int(cnt[i])}
            out.append(cur)
    res = []
    for c in out:
        span = math.radians(max(c['bins']) - min(c['bins']) + bin_deg)
        res.append((c['range'], math.radians(sum(c['bins']) / len(c['bins'])), c['range'] * span, c['points']))
    return res


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')
        self.declare_parameter('ground_z', -1.8)
        self.declare_parameter('points_topic', '/vla/points')
        self.gz = self.get_parameter('ground_z').value
        self.pub = self.create_publisher(ObstacleArray, '/cogndn/obstacles', 10)
        self.create_subscription(PointCloud2, self.get_parameter('points_topic').value, self.cb,
                                 qos_profile_sensor_data)

    def cb(self, msg):
        out = ObstacleArray()
        out.header = msg.header
        for rng, brg, w, n in clusters(parse_cloud(msg), self.gz):
            out.obstacles.append(Obstacle(range=float(rng), bearing=float(brg), width=float(w), points=int(n)))
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LidarClusterNode())


if __name__ == '__main__':
    main()
