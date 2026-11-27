#!/usr/bin/env python3
"""Measure where the road surface appears in the LiDAR (sensor frame) and suggest ground_z.

Run with the simulator up and the car on an empty stretch of road:
  python3 lidar_ground_check.py
The safety envelope treats points higher than ground_z + min_h (0.25 m) as obstacles. If ground_z is
wrong, rings of road surface look like obstacles at fixed distances and cap the car's speed."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from cogndn_core.pointcloud import parse_cloud


class Check(Node):
    def __init__(self):
        super().__init__('lidar_ground_check')
        self.zs, self.n = [], 0
        self.create_subscription(PointCloud2, '/vla/points', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        xyz = parse_cloud(msg)
        r = np.hypot(xyz[:, 0], xyz[:, 1])
        m = (r > 4.0) & (r < 25.0) & np.isfinite(xyz[:, 2])
        self.zs.append(xyz[m, 2])
        self.n += 1
        if self.n == 20:
            z = np.concatenate(self.zs)
            ground = float(np.percentile(z, 50))
            print('points 4-25 m: %d   z percentiles (5/50/95): %.2f / %.2f / %.2f'
                  % (len(z), np.percentile(z, 5), ground, np.percentile(z, 95)))
            print('road surface is at z = %.2f m in the sensor frame' % ground)
            print('-> set ground_z: %.2f for safety_envelope and lidar_cluster_node in params.yaml '
                  '(currently -1.8)' % ground)
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Check())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
