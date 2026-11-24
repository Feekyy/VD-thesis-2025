#!/usr/bin/env python3
#Contour (FRI) vs. original and improved Hough detector on the ground-truth dataset, scored the way the mission uses them:
#per color the largest circle with 20 < r < 500 px is what the mission would steer to
#Usage: python3 eval_detectors.py dataset_dir output.json
import json, math, os, sys, time
import cv2, rclpy
from rclpy.parameter import Parameter
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
    #hold_frames=0: the images are independent, a ring must not be carried over from the previous image
    rclpy.init(args=['--ros-args', '-p', 'visualize:=false', '-p', 'hold_frames:=0'])
    contour = TelloCameraContour()
    hough = TelloCameraHough(parameter_overrides=[Parameter('method', value='original')])
    improved = TelloCameraHough()
    captured = {}
    hough.circles_pub.publish = lambda msg: captured.__setitem__('hough', msg.data)
    improved.circles_pub.publish = lambda msg: captured.__setitem__('hough_improved', msg.data)
    bridge = CvBridge()
    rows = []
    for m in json.load(open(os.path.join(d, 'poses.json'))):
        img = cv2.imread(os.path.join(d, m['file']))
        t0 = time.perf_counter()
        _, contour_list, _, _ = contour.detect(img)
        t_contour = (time.perf_counter() - t0) * 1000
        msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
        lists, times = {'contour': contour_list}, {'contour': t_contour}
        for name, node in (('hough', hough), ('hough_improved', improved)):
            t0 = time.perf_counter()
            node.image_callback(msg)
            times[name] = (time.perf_counter() - t0) * 1000
            lists[name] = json.loads(captured[name])
        gt = project(m)
        for method in ('contour', 'hough', 'hough_improved'):
            lst, t = lists[method], times[method]
            for color in COLORS:
                g, sel = gt[color], mission_pick(lst, color)
                vis = g['status'] in ('full', 'partial', 'edge')
                row = {'file': m['file'], 'method': method, 'color': color, 'gt_status': g['status'], 'gt_r': g.get('r'),
                       'time_ms': t, 'raw_count': len(lst), 'raw_uncolored': sum(1 for c in lst if c.get('color') is None),
                       'raw_same_color': sum(1 for c in lst if c.get('color') == color)}
                if sel is not None:
                    row['sel'] = {'x': sel['x'], 'y': sel['y'], 'r': sel['r'], 'fraction': sel.get('fraction'),
                                  'stretch': sel.get('stretch', 1.0)}
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
