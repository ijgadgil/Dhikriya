#!/usr/bin/env python3
"""Print where the LiDAR sees obstacles (sensor frame). Place the car facing an obstacle ~8 m ahead:
the nearest raised return should be at bearing ~0 deg. If not, set safety_envelope yaw_deg accordingly."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from cogndn_core.pointcloud import parse_cloud


class P(Node):
    def __init__(self):
        super().__init__('lidar_check')
        self.n = 0
        self.create_subscription(PointCloud2, '/vla/points', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        xyz = parse_cloud(m)
        r = np.hypot(xyz[:, 0], xyz[:, 1])
        far = (r > 6) & (r < 16)
        gz = float(np.median(xyz[far, 2])) if far.any() else float('nan')
        raised = (xyz[:, 2] - gz > 0.25) & (r > 3.0)
        if raised.any():
            i = np.argmin(np.where(raised, r, np.inf))
            print('ground z %.2f | nearest raised return %.1f m at bearing %.0f deg | frame %s'
                  % (gz, r[i], np.degrees(np.arctan2(xyz[i, 1], xyz[i, 0])), m.header.frame_id))
        else:
            print('ground z %.2f | no raised returns' % gz)
        self.n += 1
        if self.n >= 5:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(P())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
