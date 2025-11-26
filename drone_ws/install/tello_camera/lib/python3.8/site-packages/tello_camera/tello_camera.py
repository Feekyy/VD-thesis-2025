#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import json

def clamp01(x):
    return max(0.0, min(1.0, float(x)))

class TelloCamera(Node):
    def __init__(self):
        super().__init__('image_detector_node')

        self.declare_parameter('image_topic', '/camera/raw_image')
        self.declare_parameter('expected_max_circles', 3)
        self.declare_parameter('activation_threshold', 0.1)
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
        self.circles_pub = self.create_publisher(String, 'donut_detector/circles', 10)

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
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

        self.get_logger().info(f'BlobDetectorNode started (Contour mode), subscribing to: {self.image_topic}')

    def find_color_blobs(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.medianBlur(hsv, 5) 
        
        detections = []
        total_area_ratio = 0.0
        
        image_area = cv_image.shape[0] * cv_image.shape[1]

        for color_name, ranges in self.color_ranges_hsv.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for (lower, upper) in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                curr_mask = cv2.inRange(hsv, lower_np, upper_np)
                mask = cv2.bitwise_or(mask, curr_mask)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300:
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    
                    if radius > 15 and radius < 300:
                        detections.append({
                            'x': int(x),
                            'y': int(y),
                            'r': int(radius),
                            'color': color_name,
                            'fraction': 1.0,
                            'area': area
                        })
                        
                        total_area_ratio += (area / image_area)

        return detections, clamp01(total_area_ratio * 100)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        detections, presence_score = self.find_color_blobs(cv_image)

        count = len(detections)
        
        activated = count > 0

        self.activation_pub.publish(Bool(data=activated))
        self.score_pub.publish(Float32(data=float(presence_score)))
        self.circles_pub.publish(String(data=json.dumps(detections)))

        if self.visualize:
            vis = cv_image.copy()
            for det in detections:
                x, y, r = det['x'], det['y'], det['r']
                color_val = (0, 255, 0)
                if det['color'] == 'red': color_val = (0, 0, 255)
                elif det['color'] == 'blue': color_val = (255, 0, 0)

                cv2.circle(vis, (x, y), r, color_val, 2)
                cv2.circle(vis, (x, y), 2, (0, 255, 255), 3)
                
                cv2.putText(vis, f"{det['color']}", (x - r, y - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_val, 2)
                            
            cv2.imshow('donut_detector', vis)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TelloCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()