#!/usr/bin/env python3
import os
import math
import rclpy
import cv2
import numpy as np
import json

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory

#Domains of the fuzzy inputs, the universes of the FBDL rule base (rules/donut_confidence.fbdl) have to cover them
FRI_DOMAINS = {
    'radius': (0.0, 1000.0),
    'hue_diff': (0.0, 90.0),
    'saturation': (0.0, 255.0),
    'brightness': (0.0, 255.0),
    'circularity': (0.0, 1.0),
}

#Clamps a value between 0.0 and 1.0
def clamp01(x):
    return max(0.0, min(1.0, float(x)))

#Distance of two OpenCV hue values (0-179) on the hue circle, so 2 and 177 are close to each other
def hue_distance(h: float, expected_hue: float) -> float:
    diff = abs(h - expected_hue)
    if diff > 90:
        diff = 180 - diff
    return diff

#Mean of OpenCV hue values on the hue circle (the plain mean of 2 and 177 would be 89.5, a cyan color instead of red)
def circular_mean_hue(hues) -> float:
    angles = hues.astype(np.float64) * (np.pi / 90.0)
    mean_angle = math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
    return (mean_angle * 90.0 / np.pi) % 180.0

#Fuzzy logic: membership function for ideal donut radius
def fuzzy_ideal_radius(r: float) -> float:
    if r <= 40:
        return clamp01((r - 10) / 30.0)
    if r >= 120:
        return clamp01((160 - r) / 40.0)
    return 1.0

#Fuzzy logic: membership function for large radius detection
def fuzzy_large_radius(r: float) -> float:
    return clamp01((r - 80) / 60.0)

#Fuzzy logic: membership function for small radius detection
def fuzzy_small_radius(r: float) -> float:
    return clamp01((60 - r) / 40.0)

#Fuzzy logic: calculates color strength based on the hue difference, the saturation and the brightness
def fuzzy_color_strength(hue_diff: float, s: float, v: float) -> float:
    score_h = clamp01(1.0 - hue_diff / 25.0)
    score_s = clamp01(s / 255.0)
    score_v = clamp01(v / 255.0)
    return clamp01((score_h * 0.6) + (score_s * 0.25) + (score_v * 0.15))

#The original hand-made fuzzy confidence: weighted sum of the size and the color memberships
def manual_confidence(features) -> float:
    r = features['radius']
    f_size = max(fuzzy_ideal_radius(r), fuzzy_small_radius(r)*0.5, fuzzy_large_radius(r)*0.3)
    f_color = fuzzy_color_strength(features['hue_diff'], features['saturation'], features['brightness'])
    return clamp01(0.55 * f_size + 0.45 * f_color)

#Initializes the ROS 2 node and sets up image processing parameters
class TelloCameraContour(Node):
    def __init__(self):
        super().__init__('tello_camera_contour')
        self.declare_parameter('image_topic', '/camera/raw_image')
        self.declare_parameter('visualize', True)
        self.declare_parameter('activation_threshold', 0.12)
        self.declare_parameter('expected_max_circles', 3)

        #Fuzzy method: "fri" = fuzzy rule interpolation of the University of Miskolc (FRIBE) with an FBDL rule base,
        #"manual" = the original hand-made membership functions (kept for comparison)
        self.declare_parameter('fuzzy_method', 'fri')
        self.declare_parameter('rules_file', '')
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.threshold = self.get_parameter('activation_threshold').get_parameter_value().double_value
        self.expected_max = self.get_parameter('expected_max_circles').get_parameter_value().integer_value
        self.fuzzy_method = self.get_parameter('fuzzy_method').get_parameter_value().string_value
        if self.fuzzy_method == 'fri':
            self.fri_engine = self.load_fri_engine(self.get_parameter('rules_file').get_parameter_value().string_value)
        elif self.fuzzy_method != 'manual':
            raise ValueError(f"Unknown fuzzy_method '{self.fuzzy_method}', use 'fri' or 'manual'")

        self.activation_pub = self.create_publisher(Bool, 'donut_detector/activation', 10)
        self.score_pub = self.create_publisher(Float32, 'donut_detector/activation_degree', 10)
        self.info_pub = self.create_publisher(String, 'donut_detector/info', 10)
        self.circles_pub = self.create_publisher(String, 'donut_detector/circles', 10)
        self.vis_pub = self.create_publisher(Image, 'donut_detector/visualization', 10)

        self.bridge = CvBridge()

        #Define color ranges in HSV space
        self.color_ranges = {
            'red':  ( [0, 100, 70],  [10, 255, 255],   0.0 ),
            'red2': ( [170,100,70], [179, 255, 255],  0.0 ),
            'green':([40, 60, 50],  [85, 255, 255],   60.0),
            'blue': ([90, 60, 40],  [140,255,255],   120.0),
        }
        self.get_logger().info(
            f"Contour detection started, sub: {self.image_topic}, visualize={self.visualize}, fuzzy={self.fuzzy_method}"
        )
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)

    #Loads the FBDL rule base into the FRIBE engine and checks that its universes cover every input domain
    def load_fri_engine(self, rules_file):
        try:
            from fribe.loader import load_engine_from_string
        except ImportError:
            raise RuntimeError(
                "fribe-python is missing: rebuild the docker image (start.bash -> build) or use -p fuzzy_method:=manual"
            )
        if not rules_file:
            rules_file = os.path.join(get_package_share_directory('tello_camera'), 'rules', 'donut_confidence.fbdl')
        with open(rules_file) as f:
            engine = load_engine_from_string(f.read())

        #The engine raises an error for values outside of a universe, so the two corners of the domains are tried once here
        for corner in (0, 1):
            engine.calc_consequences({name: limits[corner] for name, limits in FRI_DOMAINS.items()})
        self.get_logger().info(f"FRI rule base loaded: {rules_file}")
        return engine

    #Confidence of one candidate: how much it looks like a donut of the searched color (0-1)
    def confidence(self, features) -> float:
        if self.fuzzy_method == 'manual':
            return manual_confidence(features)
        self.fri_engine.calc_consequences(
            {name: min(max(features[name], low), high) for name, (low, high) in FRI_DOMAINS.items()}
        )
        return clamp01(self.fri_engine.get_state('confidence'))

    #Creates color masks for each color range in HSV space
    def build_color_masks(self, hsv_img):
        masks = {}
        for name, (low, high, hue) in self.color_ranges.items():
            low_np = np.array(low, dtype=np.uint8)
            high_np = np.array(high, dtype=np.uint8)
            m = cv2.inRange(hsv_img, low_np, high_np)
            masks[name] = (m, hue, name)
        return masks

    #Analyzes contours in a mask and returns detected circles with confidence scores
    def analyze_mask_contours(self, hsv_img, mask, color_label_hint, expected_hue):
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        H, W = mask.shape[:2]
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 250:
                continue
            (x,y), r = cv2.minEnclosingCircle(cnt)
            if r < 8 or r > max(W,H):
                continue

            #Only the colored pixels inside the contour, otherwise the background seen through the hole would be averaged in too
            filled = np.zeros_like(mask)
            cv2.drawContours(filled, [cnt], -1, 255, -1)
            ring_pixels = hsv_img[(filled > 0) & (mask > 0)]
            if ring_pixels.size == 0:
                continue
            mean_h = circular_mean_hue(ring_pixels[:,0])
            mean_s = float(np.mean(ring_pixels[:,1]))
            mean_v = float(np.mean(ring_pixels[:,2]))

            #Inputs of the fuzzy confidence, circularity: how much of the enclosing circle the contour fills (1 = full circle)
            features = {
                'radius': float(r),
                'hue_diff': hue_distance(mean_h, expected_hue),
                'saturation': mean_s,
                'brightness': mean_v,
                'circularity': float(area / (math.pi * r * r)),
            }
            detections.append({
                'x': int(x), 'y': int(y), 'r': int(r),
                'area': float(area),
                'color_hint': color_label_hint,
                'mean_hsv': [mean_h, mean_s, mean_v],
                'features': features,
                'confidence': float(self.confidence(features))
            })
        return detections

    #Finds the donuts on one image: every candidate, the best one per color and the overall presence degree
    def detect(self, cv_img):
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        masks = self.build_color_masks(hsv)
        #Special handling for red
        red_mask = None
        if 'red' in masks and 'red2' in masks:
            red_mask = cv2.bitwise_or(masks['red'][0], masks['red2'][0])
            masks.pop('red2', None)
            masks['red'] = (red_mask, masks['red'][1], 'red')
        all_detections = []

        #Analyze each color mask for contours
        for name, (mask, hue, label) in masks.items():
            dets = self.analyze_mask_contours(hsv, mask, label, hue)
            for d in dets:
                d['color'] = label
            all_detections += dets

        #Select best detection per color
        best_by_color = {}
        for d in all_detections:
            c = d['color']
            if c not in best_by_color or d['confidence'] > best_by_color[c]['confidence']:
                best_by_color[c] = d
        publish_list = []
        for c, d in best_by_color.items():
            publish_list.append({
                'x': d['x'], 'y': d['y'], 'r': d['r'],
                'color': c, 'fraction': d['confidence'], 'area': d['area']
            })

        #Calculate overall presence degree
        best_conf = 0.0
        total_area_ratio = 0.0
        img_area = cv_img.shape[0] * cv_img.shape[1]
        for p in publish_list:
            if p['fraction'] > best_conf:
                best_conf = p['fraction']
            total_area_ratio += (p['area'] / img_area)
        presence_degree = clamp01(min(1.0, total_area_ratio * 50.0) * 0.6 + best_conf * 0.4)
        return all_detections, publish_list, presence_degree, best_conf

    #Main image processing callback
    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        all_detections, publish_list, presence_degree, best_conf = self.detect(cv_img)
        activated = presence_degree >= self.threshold

        #Publish results
        self.activation_pub.publish(Bool(data=bool(activated)))
        self.score_pub.publish(Float32(data=float(presence_degree)))
        self.info_pub.publish(String(data=json.dumps({
            'count': len(publish_list),
            'presence_degree': float(presence_degree),
            'best_confidence': float(best_conf)
        })))
        self.circles_pub.publish(String(data=json.dumps(publish_list)))

        #Visualization
        if self.visualize:
            vis = cv_img.copy()
            for d in publish_list:
                x, y, r = int(d['x']), int(d['y']), int(d['r'])
                color = d['color']
                conf = d['fraction']
                color_bgr = (0,255,0)
                if color == 'red': color_bgr = (0,0,255)
                elif color == 'green': color_bgr = (0,200,0)
                elif color == 'blue': color_bgr = (255,0,0)

                cv2.circle(vis, (x,y), r, color_bgr, 2)
                cv2.circle(vis, (x,y), 3, (0,255,255), -1)
                txt = f"{color} {conf:.2f}"
                cv2.putText(vis, txt, (x - r, y - r - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

            bar_w = 200
            bar_h = 18
            margin = 12
            x0 = vis.shape[1] - bar_w - margin
            y0 = margin
            cv2.rectangle(vis, (x0, y0), (x0 + bar_w, y0 + bar_h), (50,50,50), -1)
            fill_w = int(bar_w * presence_degree)
            cv2.rectangle(vis, (x0, y0), (x0 + fill_w, y0 + bar_h), (0,200,200), -1)
            cv2.putText(vis, f"presence: {presence_degree:.2f}", (x0, y0 + bar_h + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220,220,220), 1)
            cv2.putText(vis, f"detected: {len(publish_list)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)
            if len(publish_list) > 0:
                best = max(publish_list, key=lambda x: x['fraction'])
                cv2.putText(vis, f"best: {best['color']} {best['fraction']:.2f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            try:
                cv2.imshow('donut_detector_fuzzy', vis)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().debug(f"cv2.imshow failed: {e}")
            try:
                self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            except CvBridgeError as e:
                self.get_logger().warn(f"vis publish failed: {e}")

#Entry point: initializes ROS 2, creates the node and starts the main loop
def main(args=None):
    rclpy.init(args=args)
    node = TelloCameraContour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
