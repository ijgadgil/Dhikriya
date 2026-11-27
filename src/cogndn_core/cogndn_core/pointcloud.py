"""sensor_msgs/PointCloud2 -> (N, 3) numpy array (duck-typed, no ROS import)."""
import numpy as np


def parse_cloud(msg):
    offs = {f.name: f.offset for f in msg.fields}
    if not all(k in offs for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = int(msg.width) * int(msg.height)
    fmt = '>f4' if msg.is_bigendian else '<f4'
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [fmt] * 3,
                   'offsets': [offs['x'], offs['y'], offs['z']], 'itemsize': int(msg.point_step)})
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]
