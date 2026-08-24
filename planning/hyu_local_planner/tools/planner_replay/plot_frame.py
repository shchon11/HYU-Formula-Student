#!/usr/bin/env python3
"""plot_frame.py <jsonl> <t> [<t> ...] -> PNGs of cones (ego frame) + planned path at the frame nearest each t."""
import json, sys, math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

def load(path):
    return [json.loads(l) for l in open(path) if '"ev":"frame"' in l]

def nearest(frames, t):
    return min(frames, key=lambda f: abs(f["t"] - t))

def plot(f, out):
    fig, ax = plt.subplots(figsize=(9, 9))
    cols = {"blue": "tab:blue", "yellow": "gold", "orange": "tab:orange", "big_orange": "darkorange", "unknown": "gray"}
    for k, c in cols.items():
        pts = f.get(k, [])
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], c=c, s=40, edgecolors="k", linewidths=0.5, label=f"{k} ({len(pts)})", zorder=3)
    if f.get("valid"):
        wp = f["wp"]
        ax.plot([w[0] for w in wp], [w[1] for w in wp], "-", color="red", lw=2, zorder=4, label="path")
        ax.scatter([wp[0][0]], [wp[0][1]], c="red", marker="s", s=60, zorder=5)
        for w in wp[::4]:
            ax.arrow(w[0], w[1], 0.4*math.cos(w[2]), 0.4*math.sin(w[2]), head_width=0.15, color="red", zorder=5)
    ax.arrow(0, 0, 1.0, 0, head_width=0.3, color="black", zorder=6)
    ax.add_patch(plt.Rectangle((-1, -8), 21, 16, fill=False, ls="--", color="gray"))
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlim(-6, 24); ax.set_ylim(-12, 12)
    ax.set_title(f"t={f['t']:.2f}s valid={f.get('valid')} kind={f.get('kind')} reason={f.get('reason','')}\nmap_n={f.get('map_n')} live_used={f.get('live_used')} live_added={f.get('live_added')} v={f.get('v')} status={f.get('status')}", fontsize=9)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(out, dpi=90); plt.close(fig)

if __name__ == "__main__":
    frames = load(sys.argv[1])
    base = sys.argv[1].rsplit(".", 1)[0]
    for t in sys.argv[2:]:
        f = nearest(frames, float(t))
        out = f"{base}_t{float(t):07.2f}.png"
        plot(f, out); print(out)
