#!/usr/bin/env python3
import rclpy
import json
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String
from cv_bridge import CvBridge, CvBridgeError

#Colors of the drawn rings (BGR)
DRAW_COLORS = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}

#Sample directions for checking a circle on a mask
ANGLES = np.linspace(0, 2 * np.pi, 120, endpoint=False)
COS, SIN = np.cos(ANGLES), np.sin(ANGLES)


#Clamps a value between 0.0 and 1.0
def clamp01(x):
    return max(0.0, min(1.0, float(x)))

#Detects circles in a grayscale image using Hough Circle Transform (original method)
def detect_circles(gray, dp=1.2, min_dist=50, param1=100, param2=40, min_radius=20, max_radius=300):
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=min_dist,
                               param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius)
    if circles is None:
        return []
    circles = np.uint16(np.around(circles))
    return circles[0, :].tolist()

#Calculates color fraction in the ring area of a detected circle (original method)
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

#Share of the points of a circle that fall on the mask (None if most of the circle is outside of the image)
def hit_fraction(mask, x, y, radius):
    h, w = mask.shape
    xs = np.round(x + radius * COS).astype(int)
    ys = np.round(y + radius * SIN).astype(int)
    inside = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    if inside.sum() < len(ANGLES) * 0.25:
        return None
    return float((mask[ys[inside], xs[inside]] > 0).mean())

#Outer radius of the ring from the radial profile of the mask, and the colored share of the ring band (0.6R-R)
#The Hough may lock onto the inner or the outer edge of the ring, this always gives the outer one
def refine_circle(mask, x, y, r):
    profile = []
    for radius in np.arange(max(1.5, 0.3 * r), 2.0 * r + 1, max(0.5, r / 50)):
        f = hit_fraction(mask, x, y, radius)
        if f is not None:
            profile.append((radius, f))
    if not profile:
        return None

    #From the most colored radius outwards, the outer edge is where the colored share drops below half
    i_peak = max(range(len(profile)), key=lambda i: profile[i][1])
    peak = profile[i_peak][1]
    outer = profile[i_peak][0]
    for radius, f in profile[i_peak:]:
        if f < 0.5 * peak:
            break
        outer = radius

    band = [hit_fraction(mask, x, y, radius) for radius in np.linspace(0.62 * outer, 0.96 * outer, 8)]
    band = [b for b in band if b is not None]
    return outer, (float(np.mean(band)) if band else 0.0)

#Improved method: Hough circles on the color mask of one color, also on horizontally stretched copies of it
#The rings stand vertically and the drone does not roll, so from the side a ring is a tall ellipse:
#stretched horizontally by the right factor it becomes a circle again
#Returns the best ring (in the coordinates of the mask) or None
def find_ring(mask, scale, param2, min_fraction, stretches, max_radius, pad=0.2, top=4):
    if cv2.countNonZero(mask) < 40:
        return None

    #The Hough only runs around the colored pixels
    bx, by, bw, bh = cv2.boundingRect(mask)
    h, w = mask.shape
    margin = int(0.25 * max(bw, bh)) + 4
    x0, y0 = max(0, bx - margin), max(0, by - margin)
    x1, y1 = min(w, bx + bw + margin), min(h, by + bh + margin)
    roi = mask[y0:y1, x0:x1]

    #The empty border lets the Hough find rings whose center is outside of the image
    pad_px = int(round(pad * w))
    best = None
    for s in stretches:
        full = mask if s == 1.0 else cv2.resize(mask, None, fx=s, fy=1.0, interpolation=cv2.INTER_NEAREST)
        part = roi if s == 1.0 else cv2.resize(roi, None, fx=s, fy=1.0, interpolation=cv2.INTER_NEAREST)
        img = cv2.GaussianBlur(part, (5, 5), 0)
        img = cv2.copyMakeBorder(img, pad_px, pad_px, pad_px, pad_px, cv2.BORDER_CONSTANT, value=0)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1.0, minDist=max(10, 80 * scale), param1=100,
                                   param2=param2, minRadius=int(12 * scale), maxRadius=int(max_radius * scale))
        if circles is None:
            continue

        for cx, cy, cr in circles[0][:top]:
            #Back to the coordinates of the (stretched) mask
            x, y = cx - pad_px + x0 * s, cy - pad_px + y0
            refined = refine_circle(full, x, y, cr)
            if refined is None:
                continue
            radius, fraction = refined
            if fraction < min_fraction or not 20 < radius / scale < max_radius:
                continue
            if best is None or fraction > best['fraction']:
                best = {'x': x / s, 'y': y, 'r': radius, 'fraction': fraction, 'stretch': s}
    return best

#Initializes the ROS 2 node and sets up Hough circle detection parameters
class TelloCameraHough(Node):
    def __init__(self, **kwargs):
        super().__init__('tello_camera_hough', **kwargs)

        self.declare_parameter('image_topic', '/camera/raw_image')
        self.declare_parameter('expected_max_circles', 3)
        self.declare_parameter('activation_threshold', 0.25)
        self.declare_parameter('visualize', True)
        self.declare_parameter('debug', False)
        #'improved': Hough on the color masks (default), 'original': Hough on the grayscale image
        self.declare_parameter('method', 'improved')
        self.declare_parameter('scale', 0.4)
        self.declare_parameter('param2', 25)
        self.declare_parameter('min_fraction', 0.4)
        self.declare_parameter('stretches', [1.0, 1.5, 2.0, 3.0])
        self.declare_parameter('max_radius', 500)
        self.declare_parameter('hold_frames', 3)
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.expected_max = self.get_parameter('expected_max_circles').get_parameter_value().integer_value
        self.threshold = self.get_parameter('activation_threshold').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.method = self.get_parameter('method').value
        self.scale = float(self.get_parameter('scale').value)
        self.param2 = int(self.get_parameter('param2').value)
        self.min_fraction = float(self.get_parameter('min_fraction').value)
        self.stretches = [float(s) for s in self.get_parameter('stretches').value]
        self.max_radius = int(self.get_parameter('max_radius').value)
        self.hold_frames = int(self.get_parameter('hold_frames').value)
        if self.method not in ('improved', 'original'):
            raise ValueError(f"Unknown method '{self.method}', use 'improved' or 'original'")
        self.activation_pub = self.create_publisher(Bool, 'donut_detector/activation', 10)
        self.score_pub = self.create_publisher(Float32, 'donut_detector/activation_degree', 10)
        self.info_pub = self.create_publisher(String, 'donut_detector/info', 10)
        self.circles_pub = self.create_publisher(String, 'donut_detector/circles', 10)
        self.bridge = CvBridge()

        #Define color ranges in HSV (original method)
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

        #Improved method: the same color ranges as the contour detector, so only the shape detection differs
        self.mask_ranges = {
            'red': [([0, 100, 70], [10, 255, 255]), ([170, 100, 70], [179, 255, 255])],
            'green': [([40, 60, 50], [85, 255, 255])],
            'blue': [([90, 60, 40], [140, 255, 255])],
        }

        #Last ring of each color and how many frames it has been sent again for
        self.held = {}

        #The improved method is slower, so it always takes the newest image instead of working through a queue
        queue = 1 if self.method == 'improved' else 10
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, queue)

        self.get_logger().info(f'Detector started with Hough Mode ({self.method}), subscribing to: {self.image_topic}')

    #Main image processing callback
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if self.method == 'original':
            self.process_original(cv_image)
        else:
            self.process_improved(cv_image)

    #Original method: Hough on the grayscale image, then a color for every circle
    def process_original(self, cv_image):
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

    #Improved method: for every color one ring, found by the Hough on the color mask
    def process_improved(self, cv_image):
        small = cv2.resize(cv_image, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        detections = []

        for color, ranges in self.mask_ranges.items():
            mask = None
            for low, high in ranges:
                m = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            ring = find_ring(mask, self.scale, self.param2, self.min_fraction, self.stretches, self.max_radius)
            if ring is not None:
                det = {
                    'x': int(round(ring['x'] / self.scale)),
                    'y': int(round(ring['y'] / self.scale)),
                    'r': int(round(ring['r'] / self.scale)),
                    'color': color,
                    'fraction': round(ring['fraction'], 3),
                    'stretch': ring['stretch']
                }
                self.held[color] = (det, 0)
                detections.append(det)
            elif color in self.held and self.held[color][1] < self.hold_frames:
                #Short dropout: the last ring is sent again unchanged, the mission takes it as an old image and waits
                det, count = self.held[color]
                self.held[color] = (det, count + 1)
                detections.append(dict(det, held=True))
            else:
                self.held.pop(color, None)

        presence_degree = clamp01(len(detections) / max(1, self.expected_max))
        max_color_degree = max([d['fraction'] for d in detections], default=0.0)
        activation_degree = min(presence_degree, max_color_degree)
        activated = activation_degree >= self.threshold

        self.activation_pub.publish(Bool(data=activated))
        self.score_pub.publish(Float32(data=float(activation_degree)))
        self.circles_pub.publish(String(data=json.dumps(detections)))
        self.info_pub.publish(String(data=json.dumps({
            'count': len(detections),
            'presence_degree': float(presence_degree),
            'color_degree': float(max_color_degree),
            'activation_degree': float(activation_degree),
            'activated': bool(activated)
        })))

        #Visualization: the found ring as an ellipse (a stretched circle), a thin line if it is only held
        if self.visualize:
            vis = cv_image.copy()
            for det in detections:
                bgr = DRAW_COLORS[det['color']]
                axes = (int(det['r'] / det['stretch']), det['r'])
                cv2.ellipse(vis, (det['x'], det['y']), axes, 0, 0, 360, bgr, 1 if det.get('held') else 3)
                label = f"{det['color']} {det['fraction']:.2f}" + (" (held)" if det.get('held') else f" x{det['stretch']:g}")
                cv2.putText(vis, label, (det['x'] - axes[0], det['y'] - det['r'] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
            cv2.imshow('donut_detector', vis)
            cv2.waitKey(1)

#Entry point: initializes ROS 2, creates the node and starts the main loop
def main(args=None):
    rclpy.init(args=args)
    node = TelloCameraHough()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
