#!/usr/bin/env python3
import collections, json, statistics as st, sys
rows = json.load(open(sys.argv[1]))
def cat(s):
    return {'full': 'teljesen látszik', 'partial': 'részben / képszélen', 'edge': 'részben / képszélen'}.get(s, 'nem látszik')
for method in [m for m in ('contour', 'hough', 'hough_improved') if any(r['method'] == m for r in rows)]:
    R = [r for r in rows if r['method'] == method]
    print(f'== {method}')
    for c in ('teljesen látszik', 'részben / képszélen', 'nem látszik'):
        part = [r for r in R if cat(r['gt_status']) == c]
        cnt = collections.Counter(r['outcome'] for r in part)
        print(f'  {c:20} n={len(part):3d}  ' + '  '.join(f'{k}={v}' for k, v in sorted(cnt.items())))
    ok = [r for r in R if r['outcome'] == 'correct']
    print(f"  center error of correct (median px): {st.median(r['err'] for r in ok):.1f}   radius ratio r/gt_r (median): {st.median(r['sel']['r'] / r['gt_r'] for r in ok):.2f}")
    per_img = {r['file']: r for r in R}
    print(f"  time per image ms: mean={st.mean(r['time_ms'] for r in per_img.values()):.1f} median={st.median(r['time_ms'] for r in per_img.values()):.1f}")
    print(f"  raw circles per image: mean={st.mean(r['raw_count'] for r in per_img.values()):.2f} max={max(r['raw_count'] for r in per_img.values())}"
          f"  uncolored total={sum(r['raw_uncolored'] for r in per_img.values())}")
