#!/usr/bin/env python3
#Charts of the fuzzy comparison: decision surfaces and the confidence of good/bad candidates
#Usage (inside the container): python3 plots.py candidates.json donut_confidence.fbdl output_dir
import json, math, os, random, statistics as st, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.ticker import FuncFormatter
from tello_camera.tello_camera_contour import manual_confidence, FRI_DOMAINS
from fribe.loader import load_engine_from_string

cand_path, rules_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
BLUE_RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
             "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]
CMAP = LinearSegmentedColormap.from_list("blue_seq", BLUE_RAMP)
GOOD, BAD = "#2a78d6", "#eb6834"
comma = FuncFormatter(lambda v, _: f"{v:g}".replace(".", ","))

C = json.load(open(cand_path))["candidates"]
for c in C:
    c["good"] = c["gt_status"] in ("full", "partial", "edge") and c["err"] is not None and c["err"] <= max(15.0, 0.25 * c["gt_r"])
engine = load_engine_from_string(open(rules_path).read())
def fri(f):
    engine.calc_consequences({n: min(max(f.get(n, lo), lo), hi) for n, (lo, hi) in FRI_DOMAINS.items()})
    return engine.get_state("confidence")

def style(ax):
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)

#1. Decision surfaces over radius and circularity (color inputs fixed at the typical values of the data)
sat = st.median(c["new"]["saturation"] for c in C)
bri = st.median(c["new"]["brightness"] for c in C)
radii = np.geomspace(10, 600, 140)
circs = np.linspace(0.02, 1.0, 70)
fixed = {"hue_diff": 0.0, "saturation": sat, "brightness": bri}
Z_manual = np.array([[manual_confidence(dict(fixed, radius=r, circularity=k)) for r in radii] for k in circs])
Z_fri = np.array([[fri(dict(fixed, radius=r, circularity=k)) for r in radii] for k in circs])
fig, axes = plt.subplots(1, 2, figsize=(10, 4.4), dpi=200, sharey=True)
fig.set_facecolor(SURFACE)
for ax, Z, title in ((axes[0], Z_manual, "Kézi (súlyozott összeg)"), (axes[1], Z_fri, "FRI szabálybázis (FRIBE)")):
    style(ax)
    mesh = ax.pcolormesh(radii, circs, Z, cmap=CMAP, vmin=0, vmax=1, shading="auto")
    good = [c for c in C if c["good"]]; bad = [c for c in C if not c["good"]]
    ax.scatter([c["new"]["radius"] for c in good], [c["new"]["circularity"] for c in good], s=14, facecolors="none",
               edgecolors="#ffffff", linewidths=1.6, zorder=3)
    ax.scatter([c["new"]["radius"] for c in good], [c["new"]["circularity"] for c in good], s=14, facecolors="none",
               edgecolors=INK, linewidths=0.7, label="jó jelölt", zorder=4)
    ax.scatter([c["new"]["radius"] for c in bad], [c["new"]["circularity"] for c in bad], s=16, marker="x",
               color="#ffffff", linewidths=2.0, zorder=3)
    ax.scatter([c["new"]["radius"] for c in bad], [c["new"]["circularity"] for c in bad], s=16, marker="x",
               color=BAD, linewidths=1.0, label="rossz jelölt", zorder=4)
    ax.set_xscale("log")
    ax.set_xticks([10, 20, 50, 100, 200, 500])
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    ax.yaxis.set_major_formatter(comma)
    ax.set_xlim(10, 600); ax.set_ylim(0, 1.02)
    ax.set_title(title, loc="left", color=INK, fontsize=11, pad=8)
    ax.set_xlabel("Sugár (px, logaritmikus)", color=INK2, fontsize=10)
axes[0].set_ylabel("Alakjelleg (kontúr / befoglaló kör)", color=INK2, fontsize=10)
#The upper left corner of the left panel has no candidates, the legend goes there
axes[0].legend(loc="upper left", fontsize=8, frameon=True, facecolor=SURFACE, edgecolor=GRID, framealpha=0.95)
cbar = fig.colorbar(mesh, ax=axes, fraction=0.03, pad=0.02)
cbar.set_label("Bizalom", color=INK2, fontsize=10)
cbar.ax.yaxis.set_major_formatter(comma)
cbar.ax.tick_params(colors=MUTED, labelsize=9)
cbar.outline.set_visible(False)
fig.suptitle("A két fuzzy módszer döntési felülete (szín: tiszta árnyalat, medián telítettség)",
             x=0.06, ha="left", color=INK, fontsize=12)
fig.savefig(os.path.join(out, "decision_surface.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)

#2. Confidence of the good and bad candidates per method
methods = [("Eredeti\n(hibás jellemzők)", lambda c: manual_confidence(c["old"])),
           ("Kézi\n(javított jellemzők)", lambda c: manual_confidence(c["new"])),
           ("FRI", lambda c: fri(c["new"]))]
fig, ax = plt.subplots(figsize=(8, 4.4), dpi=200)
fig.set_facecolor(SURFACE); style(ax)
ax.grid(True, axis="y", color=GRID, linewidth=0.6); ax.set_axisbelow(True)
rng = random.Random(1)
labels = []
for i, (name, fn) in enumerate(methods):
    g = [fn(c) for c in C if c["good"]]; b = [fn(c) for c in C if not c["good"]]
    auc = sum((p > n) + 0.5 * (p == n) for p in g for n in b) / (len(g) * len(b))
    for vals, dx, col, lab in ((g, -0.17, GOOD, "jó jelölt"), (b, 0.17, BAD, "rossz jelölt")):
        xs = [i + dx + rng.uniform(-0.09, 0.09) for _ in vals]
        ax.scatter(xs, vals, s=9, color=col, alpha=0.55, linewidths=0, label=lab if i == 0 else None, zorder=3)
        ax.plot([i + dx - 0.12, i + dx + 0.12], [st.median(vals)] * 2, color=INK, linewidth=2, zorder=4)
    labels.append(f"{name}\nAUC = {auc:.2f}".replace(".", ","))
ax.set_xticks(range(len(methods))); ax.set_xticklabels(labels, color=INK2, fontsize=9)
ax.set_ylim(-0.03, 1.03); ax.yaxis.set_major_formatter(comma)
ax.set_ylabel("Bizalom", color=INK2, fontsize=10)
ax.set_title("A jó és a rossz jelöltek bizalma (vonal: medián)", loc="left", color=INK, fontsize=12, pad=10)
ax.legend(loc="upper left", fontsize=9, frameon=False, bbox_to_anchor=(1.0, 1.0))
fig.savefig(os.path.join(out, "confidence_separation.png"), facecolor=SURFACE, bbox_inches="tight")
plt.close(fig)
print("ok", round(sat, 1), round(bri, 1))
