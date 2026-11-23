#!/usr/bin/env python3
#Charts of the Hough vs. contour comparison (run inside the container: matplotlib + OpenCV)
#Usage: python3 plots.py detectors.json dataset_dir output_dir
import json, math, os, sys
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.ticker import FuncFormatter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gt import RINGS, project

rows_path, data_dir, out = sys.argv[1], sys.argv[2], sys.argv[3]
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
CONTOUR, HOUGH = "#2a78d6", "#eb6834"
rows = json.load(open(rows_path))
poses = {m["file"]: m for m in json.load(open(os.path.join(data_dir, "poses.json")))}

#Angle between the viewing ray and the ring axis (the rings face the world x axis): 0 = head-on
def oblique(file, color):
    m = poses[file]; (rx, ry, rz), _ = RINGS[color]
    v = (rx - m["x"], ry - m["y"], rz - m["z"])
    return math.degrees(math.acos(abs(v[0]) / math.sqrt(sum(c * c for c in v))))

#1. Share of correctly located fully visible rings by viewing angle
bins = [(0, 20, "0–20°\n(szemből)"), (20, 40, "20–40°"), (40, 90, "40° fölött\n(oldalról)")]
fig, ax = plt.subplots(figsize=(8, 4.2), dpi=200)
fig.set_facecolor(SURFACE); ax.set_facecolor(SURFACE)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
for s in ("left", "bottom"):
    ax.spines[s].set_color(AXIS)
ax.tick_params(colors=MUTED, labelsize=9)
ax.grid(True, axis="y", color=GRID, linewidth=0.6); ax.set_axisbelow(True)
width = 0.36
for k, (method, color, label) in enumerate((("contour", CONTOUR, "Kontúr + FRI"), ("hough", HOUGH, "Hough"))):
    full = [r for r in rows if r["method"] == method and r["gt_status"] == "full"]
    for i, (lo, hi, _) in enumerate(bins):
        part = [r for r in full if lo <= oblique(r["file"], r["color"]) < hi]
        ok = sum(r["outcome"] == "correct" for r in part)
        x = i + (k - 0.5) * (width + 0.04)
        ax.bar(x, 100 * ok / len(part), width, color=color, label=label if i == 0 else None, zorder=3)
        ax.text(x, 100 * ok / len(part) + 2, f"{ok}/{len(part)}", ha="center", va="bottom", fontsize=9, color=INK2)
ax.set_xticks(range(len(bins))); ax.set_xticklabels([b[2] for b in bins], color=INK2, fontsize=9)
ax.set_ylim(0, 124); ax.set_yticks([0, 25, 50, 75, 100])
ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0f}%"))
ax.set_ylabel("Jó helyen talált karika", color=INK2, fontsize=10)
ax.set_xlabel("Rálátási szög a karika tengelyéhez képest", color=INK2, fontsize=10)
ax.set_title("Teljesen látható karikák: a mission által kapott kör a valódi helyen van-e", loc="left", color=INK, fontsize=11, pad=10)
ax.legend(frameon=False, fontsize=9, loc="upper center", ncol=2)
fig.savefig(os.path.join(out, "detection_by_angle.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

#2. Example images: head-on (both right), oblique (Hough picks a wrong circle), oblique (Hough finds nothing)
by_key = {(r["file"], r["color"], r["method"]): r for r in rows}
def pick(cond):
    for r in rows:
        if r["method"] != "hough" or r["gt_status"] != "full":
            continue
        c = by_key[(r["file"], r["color"], "contour")]
        a = oblique(r["file"], r["color"])
        others = [v for k, v in project(poses[r["file"]]).items() if k != r["color"]]
        alone = all(o["status"] in ("out", "behind") for o in others)
        if cond(r, c, a) and 70 <= r["gt_r"] <= 200 and (alone or a < 12):
            return r, c, a
examples = [
    (pick(lambda h, c, a: h["outcome"] == "correct" and c["outcome"] == "correct" and a < 12), "Szemből: mindkettő jó"),
    (pick(lambda h, c, a: h["outcome"] == "wrong_place" and c["outcome"] == "correct" and a > 45), "Oldalról: a Hough rossz kört ad"),
    (pick(lambda h, c, a: h["outcome"] == "missed" and c["outcome"] == "correct" and a > 55), "Oldalról: a Hough nem talál kört"),
]
fig, axes = plt.subplots(1, 3, figsize=(12, 3.6), dpi=200)
fig.set_facecolor(SURFACE)
for ax, ((h, c, a), title) in zip(axes, examples):
    img = cv2.cvtColor(cv2.imread(os.path.join(data_dir, h["file"])), cv2.COLOR_BGR2RGB)
    ax.imshow(img); ax.set_axis_off()
    g = project(poses[h["file"]])[h["color"]]
    ax.add_patch(Circle((g["u"], g["v"]), g["r"], fill=False, color="#ffffff", linewidth=1.4, linestyle="--"))
    for r, color in ((c, CONTOUR), (h, HOUGH)):
        if "sel" in r:
            s = r["sel"]
            ax.add_patch(Circle((s["x"], s["y"]), s["r"], fill=False, color=color, linewidth=2.2))
            ax.plot([s["x"]], [s["y"]], marker="+", color=color, markersize=9, markeredgewidth=2)
    ax.set_title(f"{title} ({a:.0f}°)", fontsize=10, color=INK, loc="left")
handles = [plt.Line2D([], [], color=CONTOUR, linewidth=2.2, label="Kontúr + FRI"),
           plt.Line2D([], [], color=HOUGH, linewidth=2.2, label="Hough"),
           plt.Line2D([], [], color=MUTED, linewidth=1.4, linestyle="--", label="Valódi karikahelyzet")]
fig.subplots_adjust(bottom=0.14)
fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=9, bbox_to_anchor=(0.5, 0.0))
fig.savefig(os.path.join(out, "examples.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)
print("examples:", [(e[0][0]["file"], e[0][0]["color"]) for e in examples])
