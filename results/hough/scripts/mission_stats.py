#!/usr/bin/env python3
#Summary of the full missions per detector: success, length, search, losses while tracking, how the pass-through started
#Usage: python3 mission_stats.py missions_dir   (files: <detector>_<n>.csv and mission_<detector>_<n>.log)
import csv, glob, math, os, re, statistics as st, sys

LABELS = {'contour': 'Kontúr + FRI', 'original': 'Eredeti Hough', 'improved': 'Javított Hough'}

def fmt(v, d=0):
    return f"{st.mean(v):.{d}f} ± {st.stdev(v):.{d}f}" if len(v) > 1 else f"{v[0]:.{d}f}"

def stats(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    ok = any(r['color'] == 'green' and r['phase'] == 'pass' for r in rows)
    approach = [float(r['error_x']) for r in rows if r['phase'] == 'approach']
    #The ring was big in the image (r > 100 px) while the drone was tracking it, and in the next image it was gone
    lost = sum(1 for a, b in zip(rows, rows[1:])
               if b['phase'] == 'search' and a['phase'] in ('align', 'approach', 'realign') and float(a['r'] or 0) > 100)
    t = [float(r['time']) for r in rows]
    return {'ok': ok, 'steps': int(rows[-1]['frame']), 'search': sum(r['phase'] == 'search' for r in rows),
            'lost': lost, 'rms': math.sqrt(st.mean(e * e for e in approach)) if approach else float('nan'),
            'seconds': t[-1] - t[0]}

def main():
    d = sys.argv[1]
    print("| Detektor | n | Sikeres | Misszió hossza (lépés) | Ebből keresés | Elvesztette követés közben | RMS yaw-hiba közelítéskor (px) | Átrepülés indítása: rendes / tartalék ág | Idő (s) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for det in ('contour', 'original', 'improved'):
        files = sorted(glob.glob(os.path.join(d, f"{det}_*.csv")))
        if not files:
            continue
        S = [stats(p) for p in files]
        normal = fallback = 0
        for p in files:
            log = os.path.join(d, 'mission_' + os.path.basename(p)[:-4] + '.log')
            if os.path.exists(log):
                text = open(log, errors='replace').read()
                normal += len(re.findall(r'close enough', text))
                fallback += len(re.findall(r'Forcing pass-through', text))
        print(f"| {LABELS[det]} | {len(S)} | {sum(s['ok'] for s in S)}/{len(S)} | {fmt([s['steps'] for s in S])} | {fmt([s['search'] for s in S])}"
              f" | {fmt([s['lost'] for s in S], 1)} | {fmt([s['rms'] for s in S], 1)} | {normal} / {fallback} | {fmt([s['seconds'] for s in S])} |")
    print()
    for p in sorted(glob.glob(os.path.join(d, '*.csv'))):
        s = stats(p)
        print(f"{os.path.basename(p):16s} ok={s['ok']} steps={s['steps']} search={s['search']} lost={s['lost']} rms={s['rms']:.1f} t={s['seconds']:.0f}s")

main()
