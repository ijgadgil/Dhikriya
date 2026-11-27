"""NDN consumer node: expresses the compiler's Interests through the vehicle NDNd forwarder,
verifies every segment against the trust policy and stores the result in Redis (C2 context).
python-ndn runs its own asyncio loop in a background thread."""
import asyncio
import os
import threading
import time

import rclpy
from ndn.app import NDNApp
from ndn.security import KeychainDigest
from ndn.transport.stream_face import UnixFace
from rclpy.node import Node
from std_msgs.msg import Header

from cogndn_core.rosutil import JsonLog, redis_store
from cogndn_msgs.msg import DataArrival, InterestBatch

from .fetcher import fetch
from .paths import trust_path
from .security import TrustPolicy


class ConsumerNode(Node):
    def __init__(self):
        super().__init__('ndn_consumer_node')
        self.declare_parameter('socket', '/run/cogndn/veh.sock')
        self.declare_parameter('keys', os.path.expanduser('~/.cogndn/keys'))
        self.declare_parameter('window', 8)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.policy = TrustPolicy(trust_path(), p('keys'))
        self.store, _ = redis_store(self)
        self.window = p('window')
        self.inflight = set()
        self.pub = self.create_publisher(DataArrival, '/cogndn/data', 50)
        self.log = JsonLog('consumer')
        self.loop = None
        self.app = NDNApp(face=UnixFace(p('socket')), keychain=KeychainDigest())
        threading.Thread(target=self._ndn_thread, daemon=True).start()
        self.create_subscription(InterestBatch, '/cogndn/interests', self._batch, 10)

    def _ndn_thread(self):
        async def ready():
            self.loop = asyncio.get_running_loop()
            self.get_logger().info('connected to the vehicle forwarder')
        self.app.run_forever(after_start=ready())

    def _batch(self, b):
        if self.loop is None:
            self.get_logger().warn('NDN face not ready; dropping %d Interests' % len(b.interests))
            return
        for it in b.interests:
            if it.name in self.inflight:
                continue
            self.inflight.add(it.name)
            asyncio.run_coroutine_threadsafe(self._get(it), self.loop)

    async def _get(self, it):
        t_req = time.time()
        err, item, st = '', None, {}
        try:
            item, st = await fetch(self.app, it.name, self.policy, lifetime_ms=int(max(1.0, it.lifetime_s) * 1000),
                                   window=self.window)
            self.store.put(item, now=time.time())
        except Exception as e:  # timeouts / nacks are logged as results
            err = type(e).__name__
        finally:
            self.inflight.discard(it.name)
        m = DataArrival(header=Header(frame_id='world'), name=it.name, error=err)
        m.header.stamp = self.get_clock().now().to_msg()
        if item is not None:
            m.sig_ok, m.signer, m.digest = item.sig_ok, item.signer, item.digest
            m.latency_s, m.segments, m.fresh_until = float(st['latency_s']), int(st['segments']), item.fresh_until
        self.pub.publish(m)
        self.log.write(name=it.name, source=it.source, t_req=t_req, ok=item is not None, sig_ok=m.sig_ok,
                       signer=m.signer, latency_s=m.latency_s, segments=m.segments,
                       bytes=st.get('bytes', 0), retx=st.get('retx', 0), fresh_until=m.fresh_until, error=err,
                       deadline=it.deadline)


def main():
    rclpy.init()
    rclpy.spin(ConsumerNode())


if __name__ == '__main__':
    main()
