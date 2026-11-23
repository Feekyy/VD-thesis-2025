#!/usr/bin/env python3
#Contour (FRI) vs. Hough detector on the ground-truth dataset, scored the way the mission uses them:
#per color the largest circle with 20 < r < 500 px is what the mission would steer to
import json, math, os, sys, time
import cv2, rclpy
from cv_bridge import CvBridge
sys.path.insert(0, os.path.dirname(__file__))
from gt import project
from tello_camera.tello_camera_contour import TelloCameraContour
from tello_camera.tello_camera_hough import TelloCameraHough

COLORS = ('blue', 'red', 'green')

def mission_pick(circles, color):
    cands = [c for c in circles if c.get('color') == color and 20 < c.get('r', 0) < 500]
    return max(cands, key=lambda c: c['r']) if cands else None

def main():
    d, out = sys.argv[1], sys.argv[2]
    rclpy.init(args=['--ros-args', '-p', 'visualize:=false'])
    contour = TelloCameraContour()
    hough = TelloCameraHough()
    captured = {}
    hough.circles_pub.publish = lambda msg: captured.__setitem__('circles', msg.data)
    bridge = CvBridge()
    rows = []
    for m in json.load(open(os.path.join(d, 'poses.json'))):
        img = cv2.imread(os.path.join(d, m['file']))
        t0 = time.perf_counter()
        _, contour_list, _, _ = contour.detect(img)
        t_contour = (time.perf_counter() - t0) * 1000
        msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        t0 = time.perf_counter()
        hough.image_callback(msg)
        t_hough = (time.perf_counter() - t0) * 1000
        hough_list = json.loads(captured['circles'])
        gt = project(m)
        for method, lst, t in (('contour', contour_list, t_contour), ('hough', hough_list, t_hough)):
            for color in COLORS:
                g, sel = gt[color], mission_pick(lst, color)
                vis = g['status'] in ('full', 'partial', 'edge')
                row = {'file': m['file'], 'method': method, 'color': color, 'gt_status': g['status'], 'gt_r': g.get('r'),
                       'time_ms': t, 'raw_count': len(lst), 'raw_uncolored': sum(1 for c in lst if c.get('color') is None),
                       'raw_same_color': sum(1 for c in lst if c.get('color') == color)}
                if sel is not None:
                    row['sel'] = {'x': sel['x'], 'y': sel['y'], 'r': sel['r'], 'fraction': sel.get('fraction')}
                    if vis:
                        err = math.hypot(sel['x'] - g['u'], sel['y'] - g['v'])
                        row['err'] = err
                        row['outcome'] = 'correct' if err <= max(15.0, 0.25 * g['r']) else 'wrong_place'
                    else:
                        row['outcome'] = 'false_detection'
                else:
                    row['outcome'] = 'missed' if vis else 'nothing'
                rows.append(row)
    json.dump(rows, open(out, 'w'))
    print('rows:', len(rows))

main()
