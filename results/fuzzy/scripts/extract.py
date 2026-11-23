#!/usr/bin/env python3
#Extracts every candidate contour of every image with the new (fixed) and the old features plus the ground truth
import json, math, os, sys, time
import cv2, numpy as np
import rclpy
sys.path.insert(0, os.path.dirname(__file__))
from gt import project
from tello_camera.tello_camera_contour import TelloCameraContour, hue_distance, circular_mean_hue

def old_features(hsv, filled, r, expected_hue):
    px = hsv[filled > 0]
    h = float(np.mean(px[:, 0])); s = float(np.mean(px[:, 1])); v = float(np.mean(px[:, 2]))
    return {'radius': float(r), 'hue_diff': hue_distance(h, expected_hue), 'saturation': s, 'brightness': v}

def main():
    d, out = sys.argv[1], sys.argv[2]
    rclpy.init(args=['--ros-args', '-p', 'fuzzy_method:=manual'])
    node = TelloCameraContour()
    rows = []
    mismatches = 0
    for m in json.load(open(os.path.join(d, 'poses.json'))):
        img = cv2.imread(os.path.join(d, m['file']))
        node_dets, _, _, _ = node.detect(img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        masks = node.build_color_masks(hsv)
        masks['red'] = (cv2.bitwise_or(masks['red'][0], masks.pop('red2')[0]), 0.0, 'red')
        gt = project(m)
        mine = []
        for name, (mask, hue, label) in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < 250:
                    continue
                (x, y), r = cv2.minEnclosingCircle(c)
                if r < 8 or r > 960:
                    continue
                filled = np.zeros_like(mask); cv2.drawContours(filled, [c], -1, 255, -1)
                ring = hsv[(filled > 0) & (mask > 0)]
                new = {'radius': float(r), 'hue_diff': hue_distance(circular_mean_hue(ring[:, 0]), hue),
                       'saturation': float(ring[:, 1].mean()), 'brightness': float(ring[:, 2].mean()),
                       'circularity': float(area / (math.pi * r * r))}
                g = gt[label]
                err = math.hypot(x - g['u'], y - g['v']) if 'u' in g else None
                rows.append({'file': m['file'], 'kind': m['kind'], 'color': label, 'x': int(x), 'y': int(y), 'r': int(r),
                             'area': float(area), 'new': new, 'old': old_features(hsv, filled, r, hue),
                             'gt_status': g['status'], 'gt_u': g.get('u'), 'gt_v': g.get('v'), 'gt_r': g.get('r'), 'err': err})
                mine.append((label, int(x), int(y), int(r), round(new['circularity'], 6)))
        theirs = sorted((dd['color'], dd['x'], dd['y'], dd['r'], round(dd['features']['circularity'], 6)) for dd in node_dets)
        if sorted(mine) != theirs:
            mismatches += 1
    json.dump({'gt': {mm['file']: project(mm) for mm in json.load(open(os.path.join(d, 'poses.json')))}, 'candidates': rows},
              open(out, 'w'))
    print('candidates:', len(rows), 'images:', len(set(r['file'] for r in rows)), 'mismatch vs node.detect:', mismatches)

main()
