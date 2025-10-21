#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

class CamView(Node):
    def __init__(self):
        super().__init__('cam_view')
        self.declare_parameter('image_topic', '/sim/camera/image_raw')
        topic = self.get_parameter('image_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # kamera témáknál bevett
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.get_logger().info(f'Feliratkozva: {topic}')

        # időzítő az imshow frissítéséhez (waitKey miatt)
        self.timer = self.create_timer(0.03, lambda: cv2.waitKey(1))

    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Gazebo Camera', frame)
        except Exception as e:
            self.get_logger().warn(f'Konverziós hiba: {e}')

def main():
    rclpy.init()
    node = CamView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
