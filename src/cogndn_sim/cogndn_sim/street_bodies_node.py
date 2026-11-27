"""Street actors: keeps each pedestrian's collision body inside its animated actor, and makes oncoming
traffic yield to the ego car (VelocityControl moves them regardless of contact, so without this an oncoming
car would push straight through the Prius).

Pedestrian bodies:

Gazebo actors are rendered (camera and GPU LiDAR see them) but have no physics. Each pedestrian therefore also
has a dynamic capsule `<id>_body` with VelocityControl + OdometryPublisher. This node runs the same script as
the actor (cogndn_sim.street) on simulation time and closes the loop on the body's odometry:
v_cmd = v_script + k * (p_script - p_body), rotated into the body frame, with yaw held at zero.
If the car pushes a body away, the controller walks it back onto the script.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

import yaml

from cogndn_core.paths import sim_config
from cogndn_core.geometry import odom_to_world
from cogndn_sim import street

K_POS, V_MAX = 1.5, 3.0
YIELD_AHEAD_M, YIELD_LATERAL_M, BRAKE = 20.0, 3.0, 3.0   # traffic stops if the car is this close ahead


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StreetBodies(Node):
    def __init__(self):
        super().__init__('street_bodies')
        self.declare_parameter('scenario', 's0_straight_control')
        self.declare_parameter('odom_is_world', False)
        scen = self.get_parameter('scenario').value
        self.is_world = self.get_parameter('odom_is_world').value
        st = street.load(sim_config())
        self.peds = []
        if street.active(st, scen, 'pedestrians'):
            for p in st.get('pedestrians', []):
                wp = street.waypoints(p)
                d = {'id': p['id'], 'wp': wp, 'spawn': (wp[0][1], wp[0][2]), 'pose': None,
                     'pub': self.create_publisher(Twist, '/street/%s/cmd_vel' % p['id'], 10)}
                self.create_subscription(Odometry, '/street/%s/odom' % p['id'], lambda m, d=d: self._odom(d, m), 10)
                self.peds.append(d)
        self.traffic, self.ego = [], None
        if street.active(st, scen, 'traffic'):
            for v in st.get('traffic', []):
                x, y, yaw = v['pose']
                self.traffic.append({'id': v['id'], 'p0': (x, y), 'dir': (math.cos(yaw), math.sin(yaw)),
                                     'v_set': float(v['speed']), 'v': float(v['speed']), 'd': 0.0,
                                     'pub': self.create_publisher(Twist, '/street/%s/cmd_vel' % v['id'], 10)})
            sc = yaml.safe_load(open('%s/scenarios/%s.yaml' % (sim_config(), scen)))
            self.spawn = tuple(sc['spawn'])
            self.create_subscription(Odometry, '/vla/odom', self._ego, 10)
        self.t_prev = None
        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info('pedestrian bodies: %s; yielding traffic: %s'
                               % ([d['id'] for d in self.peds], [c['id'] for c in self.traffic]))

    def _odom(self, d, m):
        p = m.pose.pose.position
        x, y = (p.x, p.y) if self.is_world else (d['spawn'][0] + p.x, d['spawn'][1] + p.y)
        d['pose'] = (x, y, _yaw(m.pose.pose.orientation))

    def _ego(self, m):
        p = m.pose.pose.position
        self.ego = odom_to_world(self.spawn, p.x, p.y, _yaw(m.pose.pose.orientation))[:2]

    def _traffic(self, t):
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        for c in self.traffic:
            c['d'] += c['v'] * dt                                   # open-loop position along its lane
            x, y = c['p0'][0] + c['dir'][0] * c['d'], c['p0'][1] + c['dir'][1] * c['d']
            want = c['v_set']
            if self.ego is not None:
                ex, ey = self.ego[0] - x, self.ego[1] - y
                ahead = ex * c['dir'][0] + ey * c['dir'][1]
                lateral = abs(-ex * c['dir'][1] + ey * c['dir'][0])
                if 0.0 < ahead < YIELD_AHEAD_M and lateral < YIELD_LATERAL_M:
                    want = 0.0
            c['v'] = max(want, c['v'] - BRAKE * dt) if want < c['v'] else min(want, c['v'] + BRAKE * dt)
            msg = Twist()
            msg.linear.x = c['v']
            c['pub'].publish(msg)

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.traffic:
            self._traffic(t)
        for d in self.peds:
            (px, py), (vx, vy) = street.state_at(d['wp'], t)
            cmd = Twist()
            yaw = 0.0
            if d['pose'] is not None:
                x, y, yaw = d['pose']
                vx, vy = vx + K_POS * (px - x), vy + K_POS * (py - y)
                cmd.angular.z = -2.0 * math.atan2(math.sin(yaw), math.cos(yaw))
            n = math.hypot(vx, vy)
            if n > V_MAX:
                vx, vy = vx * V_MAX / n, vy * V_MAX / n
            c, s = math.cos(yaw), math.sin(yaw)
            cmd.linear.x, cmd.linear.y = c * vx + s * vy, -s * vx + c * vy
            d['pub'].publish(cmd)


def main():
    rclpy.init()
    n = StreetBodies()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
