"""LiDAR time-to-collision safety envelope - the only publisher of the Prius cmd_vel.
Depends on nothing but LiDAR, odometry and the nominal command (no Redis, NDN or VLA)."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from cogndn_core.envelope import EnvelopeParams, TTCEnvelope
from cogndn_core.pointcloud import parse_cloud
from cogndn_core.rosutil import JsonLog
from cogndn_msgs.msg import EnvelopeEvent


class EnvelopeNode(Node):
    def _odom(self, m):
        self.v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)

    def __init__(self):
        super().__init__('safety_envelope')
        ep = EnvelopeParams()
        for k, v in ep.__dict__.items():
            self.declare_parameter(k, v)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('nominal_timeout_s', 0.5)
        self.env = TTCEnvelope(EnvelopeParams(**{k: self.get_parameter(k).value for k in ep.__dict__}))
        self.nom, self.t_nom, self.v = (0.0, 0.0), None, 0.0
        self.create_subscription(PointCloud2, '/vla/points', lambda m: self.env.on_cloud(time.time(), parse_cloud(m)),
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cogndn/cmd_nominal', self._nom, 10)
        self.create_subscription(Odometry, '/vla/odom', self._odom, 10)
        self.cmd = self.create_publisher(Twist, '/model/prius_vla/cmd_vel', 10)
        self.ev = self.create_publisher(EnvelopeEvent, '/cogndn/envelope_events', 10)
        self.log = JsonLog('envelope')
        self.last_kind, self.min_d = 'none', float('inf')
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._tick)

    def _nom(self, m):
        self.nom, self.t_nom = (m.linear.x, m.angular.z), time.time()

    def _tick(self):
        now = time.time()
        vn, wn = self.nom if self.t_nom and now - self.t_nom < self.get_parameter('nominal_timeout_s').value \
            else (0.0, 0.0)
        v, w, ev = self.env.apply(now, vn, wn, self.v)
        out = Twist()
        out.linear.x, out.angular.z = float(v), float(w)
        self.cmd.publish(out)
        if self.env.d is not None:
            self.min_d = min(self.min_d, self.env.d)
        kind = ev['kind'] if ev else 'none'
        if kind != self.last_kind:
            m = EnvelopeEvent(header=Header(frame_id='base_link'), kind=kind,
                              distance=float(ev.get('distance', -1)) if ev else -1.0,
                              ttc=float(ev.get('ttc', -1)) if ev else -1.0, v_cap=float(v))
            m.header.stamp = self.get_clock().now().to_msg()
            self.ev.publish(m)
            self.log.write(kind=kind, detail=ev, v_nominal=vn, v_out=v, min_distance=self.min_d)
            self.last_kind = kind


def main():
    rclpy.init()
    rclpy.spin(EnvelopeNode())


if __name__ == '__main__':
    main()
