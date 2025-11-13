#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def clamp01(x):
    return max(0.0, min(1.0, float(x)))

def detect_circles(gray, dp=1.2, min_dist=50, param1=100, param2=30, min_radius=10, max_radius=0):
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
                               param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius)
    if circles is None:
        return []
    circles = np.uint16(np.around(circles))
    return circles[0, :].tolist()

def color_fraction_in_circle(bgr_img, circle, color_ranges_hsv):
    x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
    h, w = bgr_img.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, (x, y), r, 255, thickness=-1)
    hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
    best_color = None
    best_fraction = 0.0
    for name, ranges in color_ranges_hsv.items():
        mask_color = np.zeros_like(mask)
        for low, high in ranges:
            low_np = np.array(low, dtype=np.uint8)
            high_np = np.array(high, dtype=np.uint8)
            m = cv2.inRange(hsv, low_np, high_np)
            mask_color = cv2.bitwise_or(mask_color, m)
        keep = cv2.bitwise_and(mask_color, mask)
        count_color = int(cv2.countNonZero(keep))
        count_circle = int(cv2.countNonZero(mask))
        frac = (count_color / count_circle) if count_circle > 0 else 0.0
        if frac > best_fraction:
            best_fraction = frac
            best_color = name
    return best_color, clamp01(best_fraction)

class ImageDetectorNode(Node):
    def __init__(self):
        super().__init__('image_detector_node')

        self.declare_parameter('image_topic', '/camera/raw_image')
        self.declare_parameter('expected_max_circles', 3)
        self.declare_parameter('activation_threshold', 0.5)
        self.declare_parameter('visualize', True)
        self.declare_parameter('debug', False)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.expected_max = self.get_parameter('expected_max_circles').get_parameter_value().integer_value
        self.threshold = self.get_parameter('activation_threshold').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.bridge = CvBridge()

        self.activation_pub = self.create_publisher(Bool, 'donut_detector/activation', 10)
        self.score_pub = self.create_publisher(Float32, 'donut_detector/activation_degree', 10)
        self.info_pub = self.create_publisher(String, 'donut_detector/info', 10)

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.color_ranges_hsv = {
            'red': [([0, 80, 60], [10, 255, 255]), ([170, 80, 60], [179, 255, 255])],
            'green': [([40, 50, 50], [85, 255, 255])],
            'blue': [([95, 60, 60], [130, 255, 255])],
            'yellow': [([20, 80, 80], [35, 255, 255])]
        }

        self.get_logger().info(f'ImageDetectorNode started, subscribing to: {self.image_topic}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        small = cv2.pyrDown(cv_image)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = detect_circles(gray, dp=1.2, min_dist=40, param1=100, param2=30, min_radius=5, max_radius=0)

        scale_x = cv_image.shape[1] / float(small.shape[1])
        scale_y = cv_image.shape[0] / float(small.shape[0])
        scaled_circles = []
        for c in circles:
            x = int(c[0] * scale_x)
            y = int(c[1] * scale_y)
            r = int(c[2] * ((scale_x + scale_y) / 2.0))
            scaled_circles.append((x, y, r))

        count = len(scaled_circles)
        presence_degree = clamp01(count / max(1, self.expected_max))

        detections = []
        max_color_degree = 0.0
        for c in scaled_circles:
            best_color, frac = color_fraction_in_circle(cv_image, c, self.color_ranges_hsv)
            detections.append({'circle': c, 'color': best_color, 'fraction': frac})
            if frac > max_color_degree:
                max_color_degree = frac

        color_degree = max_color_degree

        activation_degree = min(presence_degree, color_degree)
        activated = activation_degree >= self.threshold
        self.activation_pub.publish(Bool(data=activated))
        self.score_pub.publish(Float32(data=float(activation_degree)))

        info_msg = {
            'count': count,
            'presence_degree': float(presence_degree),
            'color_degree': float(color_degree),
            'activation_degree': float(activation_degree),
            'activated': bool(activated),
            'detections': detections
        }
        self.info_pub.publish(String(data=str(info_msg)))

        if self.debug:
            self.get_logger().info(f"Detections: {info_msg}")

        if self.visualize:
            vis = cv_image.copy()
            for det in detections:
                (x, y, r) = det['circle']
                color = (0, 255, 0)
                cv2.circle(vis, (x, y), r, color, 2)
                label = f"{det['color']}:{det['fraction']:.2f}"
                cv2.putText(vis, label, (x - r, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(vis, f"act_deg: {activation_degree:.2f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255) if activated else (200,200,200), 2)
            cv2.imshow('donut_detector', vis)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down image_detector_node')
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
