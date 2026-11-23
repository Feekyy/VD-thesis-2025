#!/usr/bin/env python3
#Summary of the full missions per fuzzy method: success, length, search/realign/stale frames, tracking error
import csv, glob, math, os, statistics as st, sys
def fmt(v, d=0):
    return f"{st.mean(v):.{d}f} ± {st.stdev(v):.{d}f}" if len(v) > 1 else f"{v[0]:.{d}f}"
print("| Módszer | n | Sikeres | Misszió hossza (lépés) | Keresés (lépés) | Újraigazodás | RMS hiba yaw közelítéskor (px) |")
print("|---|---|---|---|---|---|---|")
for method, label in (("manual", "Kézi"), ("fri", "FRI")):
    tot, search, realign, rms, ok = [], [], [], [], 0
    files = sorted(glob.glob(os.path.join(sys.argv[1], method + "_*.csv")))
    for p in files:
        rows = list(csv.DictReader(open(p)))
        ok += any(r["color"] == "green" and r["phase"] == "pass" for r in rows)
        tot.append(int(rows[-1]["frame"]))
        search.append(sum(r["phase"] == "search" for r in rows))
        realign.append(sum(r["phase"] == "realign" for r in rows))
        ap = [float(r["error_x"]) for r in rows if r["phase"] == "approach"]
        rms.append(math.sqrt(st.mean(e * e for e in ap)))
    print(f"| {label} | {len(files)} | {ok}/{len(files)} | {fmt(tot)} | {fmt(search)} | {fmt(realign, 1)} | {fmt(rms, 1)} |")
