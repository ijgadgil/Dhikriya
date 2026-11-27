"""YOLOv10n object detection on the front camera -> /cogndn/detections."""
import time

import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

from cogndn_core.rosutil import JsonLog
from cogndn_msgs.msg import Detection, DetectionArray


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('model', 'yolov10n.pt')     # Ultralytics downloads it on first use
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('half', True)
        self.declare_parameter('image_topic', '/vla/camera/image_raw')
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.half = bool(p('half')) and self.device == 0
        self.model = YOLO(p('model'))
        self.conf = p('conf')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(DetectionArray, '/cogndn/detections', 10)
        self.create_subscription(Image, p('image_topic'), self.cb, qos_profile_sensor_data)
        self.log = JsonLog('yolo')
        self.n, self.lat = 0, []
        self.get_logger().info('YOLO on %s (half=%s)' % (self.device, self.half))

    def cb(self, msg):
        t0 = time.perf_counter()
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res = self.model(img, device=self.device, conf=self.conf, half=self.half, verbose=False)[0]
        out = DetectionArray()
        out.header = msg.header
        for b in res.boxes:
            x, y, w, h = (float(v) for v in b.xywh[0])
            out.detections.append(Detection(cls=self.model.names[int(b.cls[0])], confidence=float(b.conf[0]),
                                            x=x, y=y, w=w, h=h))
        self.pub.publish(out)
        self.lat.append(time.perf_counter() - t0)
        self.n += 1
        if self.n % 30 == 0:
            self.log.write(kind='latency', mean_ms=1000 * sum(self.lat) / len(self.lat), n=len(self.lat),
                           classes=sorted({d.cls for d in out.detections}))
            self.lat = []


def main():
    rclpy.init()
    rclpy.spin(YoloNode())


if __name__ == '__main__':
    main()
