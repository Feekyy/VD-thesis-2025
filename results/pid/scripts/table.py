import csv, glob, os, sys, math, statistics as st

#Step response metrics of the first (blue) alignment, mean ± std over runs
def metrics(path):
    rows = [r for r in csv.DictReader(open(path)) if r["color"] == "blue" and r["phase"] in ("align", "approach", "realign")]
    n = next((i for i, r in enumerate(rows) if r["phase"] == "approach"), len(rows))
    ex = [float(r["error_x"]) for r in rows[:n]]; ey = [float(r["error_y"]) for r in rows[:n]]
    def os_(e):
        s0 = math.copysign(1, e[0]); c = [abs(v) for v in e if v != 0 and math.copysign(1, v) != s0]
        return max(c) if c else 0.0
    def settle(e):
        return next(i for i in range(len(e) + 1) if all(abs(v) < 30 for v in e[i:]))
    return dict(os_x=os_(ex), os_y=os_(ey), st_x=settle(ex), st_y=settle(ey), align=n,
                iae_x=sum(map(abs, ex)), iae_y=sum(map(abs, ey)))

def fmt(vals, d=0):
    m = st.mean(vals); s = st.stdev(vals) if len(vals) > 1 else 0.0
    return f"{m:.{d}f} ± {s:.{d}f}"

def table(folder, variants, title):
    print(f"\n{title}\n")
    print("| Változat | n | Túllövés yaw (px) | Túllövés z (px) | Beállás yaw (lépés) | Beállás z (lépés) | Ráállás (lépés) | IAE yaw (px·lépés) | IAE z (px·lépés) |")
    print("|---|---|---|---|---|---|---|---|---|")
    for v, label in variants:
        ms = [metrics(p) for p in sorted(glob.glob(os.path.join(folder, v + "_*.csv")))]
        g = lambda k: [m[k] for m in ms]
        print(f"| {label} | {len(ms)} | {fmt(g('os_x'))} | {fmt(g('os_y'))} | {fmt(g('st_x'),1)} | {fmt(g('st_y'),1)} | {fmt(g('align'),1)} | {fmt(g('iae_x'))} | {fmt(g('iae_y'))} |")

V = [("P", "P (alap)"), ("Ptuned", "P (hangolt z)"), ("PD", "PD"), ("PID", "PID")]
table(os.path.join(sys.argv[1], "raw/step"), V, "STEP")
table(os.path.join(sys.argv[1], "raw/step_without_stale_skip"), [("P", "P, kezelés nélkül")], "NOSTALE")

#Full mission: frames, tracking RMS
print("\nMISSION\n")
print("| Változat | n | Sikeres | Misszió hossza (lépés) | RMS hiba yaw közelítéskor (px) | RMS hiba z közelítéskor (px) | Befagyott képkocka |")
print("|---|---|---|---|---|---|---|")
for v, label in V:
    tot, rx, ry, stale, ok = [], [], [], [], 0
    for p in sorted(glob.glob(os.path.join(sys.argv[1], "raw/mission", v + "_*.csv"))):
        rows = list(csv.DictReader(open(p)))
        ok += any(r["color"] == "green" and r["phase"] == "pass" for r in rows)
        tot.append(int(rows[-1]["frame"])); stale.append(sum(r["phase"] == "stale" for r in rows))
        ap = [r for r in rows if r["phase"] == "approach"]
        rx.append(math.sqrt(st.mean(float(r["error_x"]) ** 2 for r in ap))); ry.append(math.sqrt(st.mean(float(r["error_y"]) ** 2 for r in ap)))
    print(f"| {label} | {len(tot)} | {ok}/{len(tot)} | {fmt(tot)} | {fmt(rx,1)} | {fmt(ry,1)} | {fmt(stale)} |")
