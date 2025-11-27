#!/usr/bin/env python3
import rclpy
import json
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge, CvBridgeError


#Clamps a value between 0.0 and 1.0
def clamp01(x):
    return max(0.0, min(1.0, float(x)))

#Detects circles in a grayscale image using Hough Circle Transform
def detect_circles(gray, dp=1.2, min_dist=50, param1=100, param2=40, min_radius=20, max_radius=300):
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
                               param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius)
    if circles is None:
        return []
    circles = np.uint16(np.around(circles))
    return circles[0, :].tolist()

#Calculates color fraction in the ring area of a detected circle
def color_fraction_in_circle(bgr_img, circle, color_ranges_hsv):
    x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
    h, w = bgr_img.shape[:2]

    mask = np.zeros((h, w), dtype=np.uint8)
    ring_width = max(5, int(r * 0.3)) 
    cv2.circle(mask, (x, y), r, 255, thickness=ring_width)
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

#Initializes the ROS 2 node and sets up Hough circle detection parameters
class TelloCamera(Node):
    def __init__(self):
        super().__init__('tello_camera_hough')

        self.declare_parameter('image_topic', '/camera/raw_image')
        self.declare_parameter('expected_max_circles', 3)
        self.declare_parameter('activation_threshold', 0.25)
        self.declare_parameter('visualize', True)
        self.declare_parameter('debug', False)
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.expected_max = self.get_parameter('expected_max_circles').get_parameter_value().integer_value
        self.threshold = self.get_parameter('activation_threshold').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.activation_pub = self.create_publisher(Bool, 'donut_detector/activation', 10)
        self.score_pub = self.create_publisher(Float32, 'donut_detector/activation_degree', 10)
        self.info_pub = self.create_publisher(String, 'donut_detector/info', 10)
        self.circles_pub = self.create_publisher(String, 'donut_detector/circles', 10)
        self.bridge = CvBridge()

        #Define color ranges in HSV
        self.color_ranges_hsv = {
            'red': [
                ([0, 100, 80], [10, 255, 255]),
                ([160, 100, 80], [179, 255, 255])
            ],
            'green': [
                ([45, 50, 50], [80, 255, 255])
            ],
            'blue': [
                ([100, 50, 50], [130, 255, 255])
            ],
        }

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f'Detector started with Hough Mode, subscribing to: {self.image_topic}')

    #Main image processing callback
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        small = cv2.resize(cv_image, (cv_image.shape[1]//1, cv_image.shape[0]//1))
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = detect_circles(gray)
        scale_x = cv_image.shape[1] / float(small.shape[1])
        scale_y = cv_image.shape[0] / float(small.shape[0])
        scaled = []

        for c in circles:
            x = int(c[0] * scale_x)
            y = int(c[1] * scale_y)
            r = int(c[2] * ((scale_x + scale_y) / 2.0))
            scaled.append((x, y, r))

        count = len(scaled)
        presence_degree = clamp01(count / max(1, self.expected_max))

        detections = []
        max_color_degree = 0.0

        #Analyze each detected circle's color
        for c in scaled:
            best_color, frac = color_fraction_in_circle(cv_image, c, self.color_ranges_hsv)
            detections.append({
                'x': c[0],
                'y': c[1],
                'r': c[2],
                'color': best_color,
                'fraction': frac
            })
            
            if frac > max_color_degree:
                max_color_degree = frac

        activation_degree = min(presence_degree, max_color_degree)
        activated = activation_degree >= self.threshold

        self.activation_pub.publish(Bool(data=activated))
        self.score_pub.publish(Float32(data=float(activation_degree)))
        self.circles_pub.publish(String(data=json.dumps(detections)))

        #Publishing info
        info_msg = {
            'count': count,
            'presence_degree': float(presence_degree),
            'color_degree': float(max_color_degree),
            'activation_degree': float(activation_degree),
            'activated': bool(activated)
        }
        self.info_pub.publish(String(data=str(info_msg)))

        #Visualization
        if self.visualize:
            vis = cv_image.copy()
            for det in detections:
                x, y, r = det['x'], det['y'], det['r']
                cv2.circle(vis, (x, y), r, (0, 255, 0), 2)
                cv2.putText(vis, f"{det['color']}:{det['fraction']:.2f}",
                            (x - r, y - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow('donut_detector', vis)
            cv2.waitKey(1)

#Entry point: initializes ROS 2, creates the node and starts the main loop
def main(args=None):
    rclpy.init(args=args)
    node = TelloCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()