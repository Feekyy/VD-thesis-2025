#!/usr/bin/env python3
#Compares the confidence methods on the extracted candidates: selection per color, good/bad separation (AUC), speed
import collections, json, math, sys, time
from tello_camera.tello_camera_contour import manual_confidence, FRI_DOMAINS
from fribe.loader import load_engine_from_string

def tol(c):
    return max(15.0, 0.25 * c['gt_r'])

def load(path):
    C = json.load(open(path))['candidates']
    for c in C:
        c['good'] = c['gt_status'] in ('full', 'partial', 'edge') and c['err'] is not None and c['err'] <= tol(c)
        c['idx'] = int(c['file'][4:8])
    return C

def fri_scorer(rules_path):
    engine = load_engine_from_string(open(rules_path).read())
    def score(f):
        engine.calc_consequences({n: min(max(f.get(n, lo), lo), hi) for n, (lo, hi) in FRI_DOMAINS.items()})
        return engine.get_state('confidence')
    return score

def auc(pos, neg):
    if not pos or not neg:
        return float('nan')
    wins = sum((p > n) + 0.5 * (p == n) for p in pos for n in neg)
    return wins / (len(pos) * len(neg))

def evaluate(C, conf):
    groups = collections.defaultdict(list)
    for c in C:
        groups[(c['file'], c['color'])].append(c)
    sel_all = [max(g, key=lambda c: c['conf'][conf])['good'] for g in groups.values() if any(x['good'] for x in g)]
    sel_multi = [max(g, key=lambda c: c['conf'][conf])['good'] for g in groups.values() if len(g) > 1 and any(x['good'] for x in g)]
    a = auc([c['conf'][conf] for c in C if c['good']], [c['conf'][conf] for c in C if not c['good']])
    return sum(sel_all), len(sel_all), sum(sel_multi), len(sel_multi), a

def main():
    C = load(sys.argv[1])
    methods = {'original': lambda c: manual_confidence(c['old']), 'manual': lambda c: manual_confidence(c['new'])}
    for path in sys.argv[2:]:
        s = fri_scorer(path)
        methods['fri:' + path.split('/')[-1]] = (lambda s: lambda c: s(c['new']))(s)
    timing = {}
    for name, fn in methods.items():
        t0 = time.perf_counter()
        for c in C:
            c.setdefault('conf', {})[name] = fn(c)
        timing[name] = (time.perf_counter() - t0) / len(C) * 1e6
    for split, part in (('train (0-149)', [c for c in C if c['idx'] < 150]), ('test (150-299)', [c for c in C if c['idx'] >= 150]), ('all', C)):
        print(f'== {split}: {len(part)} candidates, good={sum(c["good"] for c in part)}')
        for name in methods:
            ok, n, okm, nm, a = evaluate(part, name)
            print(f'  {name:28} selection {ok:3d}/{n:3d}  multi-candidate {okm:2d}/{nm:2d}  AUC={a:.3f}  {timing[name]:6.1f} us/candidate')

if __name__ == '__main__':
    main()
