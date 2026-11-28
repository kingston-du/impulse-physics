#!/usr/bin/env python3
"""Turn bench output (csv) into a small svg bar chart for the README.

usage: ./build/bench > docs/bench.csv && python3 scripts/plot_bench.py docs/bench.csv docs/bench.svg

Only uses the standard library so there's nothing to pip install.
"""

import csv
import sys


def main():
    if len(sys.argv) != 3:
        print("usage: plot_bench.py in.csv out.svg")
        return 1

    rows = list(csv.DictReader(open(sys.argv[1])))
    counts = sorted({int(r["bodies"]) for r in rows})
    data = {(int(r["bodies"]), r["mode"]): float(r["step_ms"]) for r in rows}

    w, h = 640, 340
    left, right, top, bottom = 60, 20, 40, 50
    plot_w = w - left - right
    plot_h = h - top - bottom
    max_ms = max(data.values()) * 1.1

    colors = {"brute": "#e8743b", "grid": "#3a87d8"}
    group_w = plot_w / len(counts)
    bar_w = group_w * 0.35

    out = []
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
               f'font-family="Helvetica, Arial, sans-serif" font-size="12">')
    out.append(f'<rect width="{w}" height="{h}" fill="white"/>')
    out.append(f'<text x="{w / 2}" y="22" text-anchor="middle" font-size="15">'
               'Physics step time (ms, lower is better)</text>')

    # y axis grid lines
    ticks = 5
    for i in range(ticks + 1):
        v = max_ms * i / ticks
        y = top + plot_h - plot_h * i / ticks
        out.append(f'<line x1="{left}" y1="{y:.1f}" x2="{w - right}" y2="{y:.1f}" stroke="#ddd"/>')
        out.append(f'<text x="{left - 6}" y="{y + 4:.1f}" text-anchor="end">{v:.1f}</text>')

    for gi, n in enumerate(counts):
        gx = left + gi * group_w + (group_w - 2 * bar_w) / 2
        for bi, mode in enumerate(["brute", "grid"]):
            if (n, mode) not in data:
                continue
            ms = data[(n, mode)]
            bh = plot_h * ms / max_ms
            x = gx + bi * bar_w
            y = top + plot_h - bh
            out.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w - 2:.1f}" '
                       f'height="{bh:.1f}" fill="{colors[mode]}"/>')
            out.append(f'<text x="{x + bar_w / 2 - 1:.1f}" y="{y - 4:.1f}" '
                       f'text-anchor="middle" font-size="10">{ms:.2f}</text>')
        out.append(f'<text x="{left + gi * group_w + group_w / 2:.1f}" y="{h - bottom + 18}" '
                   f'text-anchor="middle">{n} bodies</text>')

    # legend
    lx = left + 10
    for mode, label in [("brute", "brute force O(n^2)"), ("grid", "spatial hash grid")]:
        out.append(f'<rect x="{lx}" y="{h - 22}" width="12" height="12" fill="{colors[mode]}"/>')
        out.append(f'<text x="{lx + 18}" y="{h - 12}">{label}</text>')
        lx += 170

    out.append("</svg>")
    with open(sys.argv[2], "w") as f:
        f.write("\n".join(out) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
