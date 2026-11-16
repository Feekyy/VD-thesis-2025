import csv, glob, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

R = sys.argv[1]
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
STYLE = {  #fixed categorical order: slot 1..4, plus a line style as secondary encoding
    "P": ("#2a78d6", "-", "P (alap)"),
    "Ptuned": ("#eb6834", "--", "P (hangolt z)"),
    "PD": ("#1baf7a", "-.", "PD"),
    "PID": ("#eda100", ":", "PID"),
}

def load(path):
    return [r for r in csv.DictReader(open(path)) if r["color"] == "blue" and r["phase"] in ("align", "approach", "realign")]

def first_alignment(rows, key):
    n = next((i for i, r in enumerate(rows) if r["phase"] == "approach"), len(rows))
    return [float(r[key]) for r in rows[:n + 5]]

def median_run(paths, key):
    scored = sorted(paths, key=lambda p: sum(abs(v) for v in first_alignment(load(p), key)))
    return scored[len(scored) // 2]

def setup(ax, title, ylabel):
    ax.set_facecolor(SURFACE)
    ax.figure.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.set_title(title, loc="left", color=INK, fontsize=12, pad=12)
    ax.set_xlabel("Szabályzási lépés (friss képkocka)", color=INK2, fontsize=10)
    ax.set_ylabel(ylabel, color=INK2, fontsize=10)
    ax.axhspan(-30, 30, color=GRID, alpha=0.5, linewidth=0)
    ax.axhline(0, color=AXIS, linewidth=1)

def label_end(ax, xs, ys, text, color):
    ax.annotate(text, (xs[-1], ys[-1]), xytext=(6, 0), textcoords="offset points", va="center", fontsize=9, color=INK2)

def response_figure(key, ylabel, title, out, variants, legend_loc):
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=200)
    setup(ax, title, ylabel)
    for v in variants:
        color, ls, name = STYLE[v]
        path = median_run(glob.glob(os.path.join(R, "raw/step", v + "_*.csv")), key)
        ys = first_alignment(load(path), key)
        xs = list(range(len(ys)))
        ax.plot(xs, ys, color=color, linestyle=ls, linewidth=2, label=name)
    ax.text(0.995, 0.97 if legend_loc == "lower right" else 0.03, "szürke sáv: ±30 px tűrés", transform=ax.transAxes, ha="right", va="top" if legend_loc == "lower right" else "bottom", fontsize=8, color=MUTED)
    ax.legend(frameon=False, fontsize=9, loc=legend_loc)
    fig.tight_layout()
    fig.savefig(os.path.join(R, out), facecolor=SURFACE)
    plt.close(fig)

response_figure("error_x", "Vízszintes hiba (px)", "Ugrásválasz – fordulás (yaw), medián futás",
                "step_response_yaw.png", ["P", "Ptuned", "PD", "PID"], "upper right")
response_figure("error_y", "Függőleges hiba (px)", "Ugrásválasz – magasság (z), medián futás",
                "step_response_z.png", ["P", "Ptuned", "PD", "PID"], "lower right")

#Effect of skipping stale frames on the same P controller
fig, ax = plt.subplots(figsize=(8, 4.2), dpi=200)
setup(ax, "A befagyott képkockák kezelésének hatása (P-szabályzó, yaw)", "Vízszintes hiba (px)")
for folder, color, ls, name in [("raw/step_without_stale_skip", "#2a78d6", "--", "kezelés nélkül"),
                               ("raw/step", "#eb6834", "-", "kezeléssel")]:
    paths = glob.glob(os.path.join(R, folder, "P_*.csv"))
    rows = [r for r in csv.DictReader(open(median_run(paths, "error_x"))) if r["color"] == "blue" and r["phase"] in ("align", "approach", "realign")]
    n = next((i for i, r in enumerate(rows) if r["phase"] == "approach"), len(rows))
    ys = [float(r["error_x"]) for r in rows[:n + 5]]
    xs = list(range(len(ys)))
    ax.plot(xs, ys, color=color, linestyle=ls, linewidth=2, label=name)
ax.legend(frameon=False, fontsize=9, loc="upper right")
fig.tight_layout()
fig.savefig(os.path.join(R, "stale_frame_effect.png"), facecolor=SURFACE)
plt.close(fig)
print("ok")
