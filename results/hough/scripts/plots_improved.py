#!/usr/bin/env python3
#Charts of the contour vs. original Hough vs. improved Hough comparison (run inside the container: matplotlib + OpenCV)
#Usage: python3 plots_improved.py detectors_v2.json dataset_dir output_dir
import json, math, os, sys
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
from matplotlib.ticker import FuncFormatter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gt import RINGS, project

rows_path, data_dir, out = sys.argv[1], sys.argv[2], sys.argv[3]
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
#Categorical slots 1-3 in fixed order (validated: CVD separation 9.2, normal-vision 27.6; the third is labelled on every bar)
SERIES = (("contour", "#2a78d6", "Kontúr + FRI"), ("hough", "#eb6834", "Eredeti Hough"), ("hough_improved", "#1baf7a", "Javított Hough"))
rows = json.load(open(rows_path))
poses = {m["file"]: m for m in json.load(open(os.path.join(data_dir, "poses.json")))}

#Angle between the viewing ray and the ring axis (the rings face the world x axis): 0 = head-on
def oblique(file, color):
    m = poses[file]; (rx, ry, rz), _ = RINGS[color]
    v = (rx - m["x"], ry - m["y"], rz - m["z"])
    return math.degrees(math.acos(abs(v[0]) / math.sqrt(sum(c * c for c in v))))

#1. Share of correctly located fully visible rings by viewing angle, three detectors
bins = [(0, 20, "0–20°\n(szemből)"), (20, 40, "20–40°"), (40, 90, "40° fölött\n(oldalról)")]
fig, ax = plt.subplots(figsize=(8.6, 4.4), dpi=200)
fig.set_facecolor(SURFACE); ax.set_facecolor(SURFACE)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
for s in ("left", "bottom"):
    ax.spines[s].set_color(AXIS)
ax.tick_params(colors=MUTED, labelsize=9)
ax.grid(True, axis="y", color=GRID, linewidth=0.6); ax.set_axisbelow(True)
width = 0.26
for k, (method, color, label) in enumerate(SERIES):
    full = [r for r in rows if r["method"] == method and r["gt_status"] == "full"]
    for i, (lo, hi, _) in enumerate(bins):
        part = [r for r in full if lo <= oblique(r["file"], r["color"]) < hi]
        ok = sum(r["outcome"] == "correct" for r in part)
        x = i + (k - 1) * (width + 0.03)
        ax.bar(x, 100 * ok / len(part), width, color=color, label=label if i == 0 else None, zorder=3)
        ax.text(x, 100 * ok / len(part) + 2, f"{ok}/{len(part)}", ha="center", va="bottom", fontsize=8, color=INK2)
ax.set_xticks(range(len(bins))); ax.set_xticklabels([b[2] for b in bins], color=INK2, fontsize=9)
ax.set_ylim(0, 124); ax.set_yticks([0, 25, 50, 75, 100])
ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0f}%"))
ax.set_ylabel("Jó helyen talált karika", color=INK2, fontsize=10)
ax.set_xlabel("Rálátási szög a karika tengelyéhez képest", color=INK2, fontsize=10)
ax.set_title("Teljesen látható karikák: a mission által kapott kör a valódi helyen van-e", loc="left", color=INK, fontsize=11, pad=10)
ax.legend(frameon=False, fontsize=9, loc="upper center", ncol=3)
fig.savefig(os.path.join(out, "detection_by_angle_improved.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

#2. Example images: head-on (all right), oblique (the original Hough gives a wrong circle), oblique (it finds nothing)
by_key = {(r["file"], r["color"], r["method"]): r for r in rows}
def pick(cond):
    for r in rows:
        if r["method"] != "hough_improved" or r["gt_status"] != "full":
            continue
        o = by_key[(r["file"], r["color"], "hough")]
        c = by_key[(r["file"], r["color"], "contour")]
        a = oblique(r["file"], r["color"])
        others = [v for k, v in project(poses[r["file"]]).items() if k != r["color"]]
        alone = all(v["status"] in ("out", "behind") for v in others)
        if cond(r, o, c, a) and 70 <= r["gt_r"] <= 200 and (alone or a < 12):
            return r, o, c, a
examples = [
    (pick(lambda i, o, c, a: i["outcome"] == o["outcome"] == c["outcome"] == "correct" and a < 12), "Szemből: mindhárom jó"),
    (pick(lambda i, o, c, a: i["outcome"] == "correct" and o["outcome"] == "wrong_place" and a > 45), "Oldalról: az eredeti rossz kört ad"),
    (pick(lambda i, o, c, a: i["outcome"] == "correct" and o["outcome"] == "missed" and a > 55), "Oldalról: az eredeti nem talál kört"),
]
fig, axes = plt.subplots(1, 3, figsize=(12, 3.25), dpi=200)
fig.set_facecolor(SURFACE)
for ax, ((i, o, c, a), title) in zip(axes, examples):
    img = cv2.cvtColor(cv2.imread(os.path.join(data_dir, i["file"])), cv2.COLOR_BGR2RGB)
    ax.imshow(img); ax.set_axis_off()
    g = project(poses[i["file"]])[i["color"]]
    ax.add_patch(Circle((g["u"], g["v"]), g["r"], fill=False, color="#ffffff", linewidth=1.4, linestyle="--"))
    for r, (method, color, _) in zip((c, o, i), SERIES):
        if "sel" not in r:
            continue
        s = r["sel"]
        #The improved detector found an ellipse: a circle in the horizontally stretched image
        w = 2 * s["r"] / s.get("stretch", 1.0)
        ax.add_patch(Ellipse((s["x"], s["y"]), w, 2 * s["r"], fill=False, color=color, linewidth=2.2))
        ax.plot([s["x"]], [s["y"]], marker="+", color=color, markersize=9, markeredgewidth=2)
    ax.set_title(f"{title} ({a:.0f}°)", fontsize=10, color=INK, loc="left")
handles = [plt.Line2D([], [], color=color, linewidth=2.2, label=label) for _, color, label in SERIES]
handles.append(plt.Line2D([], [], color=MUTED, linewidth=1.4, linestyle="--", label="Valódi karikahelyzet"))
fig.subplots_adjust(bottom=0.1)
fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=9, bbox_to_anchor=(0.5, 0.01))
fig.savefig(os.path.join(out, "examples_improved.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)
print("examples:", [(e[0][0]["file"], e[0][0]["color"]) for e in examples])
