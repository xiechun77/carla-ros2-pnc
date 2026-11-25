#!/usr/bin/env python3
"""Compare multiple PnC evaluation runs and generate Markdown table + plots.

Usage:
  python3 experiments/eval/compare.py <run_dir_1> <run_dir_2> [...] [--out-dir <dir>]
"""

import argparse
import csv
import json
import math
import os
import sys


def load_run(run_dir):
    summary_path = os.path.join(run_dir, "summary.json")
    samples_path = os.path.join(run_dir, "samples.csv")
    if not os.path.isfile(summary_path):
        raise FileNotFoundError(f"Missing summary.json in {run_dir}")
    with open(summary_path, "r", encoding="utf-8") as f:
        summary = json.load(f)

    samples = []
    if os.path.isfile(samples_path):
        with open(samples_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                parsed = {}
                for k, v in row.items():
                    if k == "goal_stopped":
                        parsed[k] = (v == "True")
                    elif v in ("", "None", None):
                        parsed[k] = None
                    else:
                        try:
                            parsed[k] = float(v)
                        except ValueError:
                            parsed[k] = v
                samples.append(parsed)

    label = f"{summary.get('planner', '?')}+{summary.get('controller', '?')} ({summary.get('scenario', '?')})"
    return {
        "dir": run_dir,
        "name": os.path.basename(os.path.normpath(run_dir)),
        "label": label,
        "summary": summary,
        "samples": samples,
    }


def fmt(val, suffix=""):
    if val is None:
        return "-"
    if isinstance(val, bool):
        return "Yes" if val else "No"
    if isinstance(val, float):
        return f"{val:.3f}{suffix}"
    return f"{val}{suffix}"


def build_markdown(runs):
    metrics = [
        ("Planner", "planner", ""),
        ("Controller", "controller", ""),
        ("Scenario", "scenario", ""),
        ("Goal Reached", "goal_reached", ""),
        ("Travel Distance (m)", "travel_distance_m", ""),
        ("Collisions", "collision_count", ""),
        ("Min Obstacle Dist (m)", "min_obstacle_dist_m", ""),
        ("Lateral Err RMSE (m)", "lateral_error_rmse_m", ""),
        ("Lateral Err Max (m)", "lateral_error_max_m", ""),
        ("Heading Err RMSE (deg)", "heading_error_rmse_deg", ""),
        ("Heading Err Max (deg)", "heading_error_max_deg", ""),
        ("Speed Err RMSE (m/s)", "speed_error_rmse_mps", ""),
        ("Lon Jerk RMS (m/s³)", "lon_jerk_rms_mps3", ""),
        ("Steer Rate RMS (1/s)", "steer_rate_rms_per_s", ""),
        ("Plan Time Mean / P95 (ms)", "_plan_time", ""),
        ("Ctrl Time Mean / P95 (ms)", "_ctrl_time", ""),
    ]

    header = "| Metric | " + " | ".join(r["label"] for r in runs) + " |"
    sep = "|---|" + "|".join(["---"] * len(runs)) + "|"
    lines = [header, sep]

    for title, key, suffix in metrics:
        cells = []
        for r in runs:
            s = r["summary"]
            if key == "_plan_time":
                cells.append(f"{fmt(s.get('planning_time_mean_ms'))} / {fmt(s.get('planning_time_p95_ms'))}")
            elif key == "_ctrl_time":
                cells.append(f"{fmt(s.get('control_time_mean_ms'))} / {fmt(s.get('control_time_p95_ms'))}")
            else:
                cells.append(fmt(s.get(key), suffix))
        lines.append(f"| {title} | " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def save_plots(runs, out_png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[compare] matplotlib not installed; skipping PNG plot generation.", file=sys.stderr)
        return False

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), dpi=150)
    ax_lat, ax_head = axes[0]
    ax_spd, ax_steer = axes[1]

    for r in runs:
        samples = r["samples"]
        if not samples:
            continue
        t = [s["t_rel"] for s in samples if s.get("t_rel") is not None]
        lat = [s["lateral_error"] for s in samples if s.get("t_rel") is not None]
        head = [
            math.degrees(s["heading_error"]) if s.get("heading_error") is not None else None
            for s in samples
            if s.get("t_rel") is not None
        ]
        spd = [s["ego_v"] for s in samples if s.get("t_rel") is not None]
        steer = [s["steer"] for s in samples if s.get("t_rel") is not None]

        ax_lat.plot(t, lat, label=r["label"], linewidth=1.4)
        ax_head.plot(t, head, label=r["label"], linewidth=1.4)
        ax_spd.plot(t, spd, label=r["label"], linewidth=1.4)
        ax_steer.plot(t, steer, label=r["label"], linewidth=1.4)

    ax_lat.set_title("Lateral Error (m)")
    ax_lat.set_xlabel("Time (s)")
    ax_lat.set_ylabel("e_y (m)")
    ax_lat.grid(True, alpha=0.3)
    ax_lat.legend()

    ax_head.set_title("Heading Error (deg)")
    ax_head.set_xlabel("Time (s)")
    ax_head.set_ylabel("e_psi (deg)")
    ax_head.grid(True, alpha=0.3)
    ax_head.legend()

    ax_spd.set_title("Ego Speed (m/s)")
    ax_spd.set_xlabel("Time (s)")
    ax_spd.set_ylabel("v (m/s)")
    ax_spd.grid(True, alpha=0.3)
    ax_spd.legend()

    ax_steer.set_title("Steering Command")
    ax_steer.set_xlabel("Time (s)")
    ax_steer.set_ylabel("steer [-1, 1]")
    ax_steer.grid(True, alpha=0.3)
    ax_steer.legend()

    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


def save_svg_plots(runs, out_svg):
    """Zero-dependency SVG 2x2 plot generator (works even without matplotlib)."""
    palette = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    panels = [
        ("Lateral Error e_y (m)", lambda s: s.get("lateral_error")),
        (
            "Heading Error e_psi (deg)",
            lambda s: math.degrees(s["heading_error"]) if s.get("heading_error") is not None else None,
        ),
        ("Ego Speed v (m/s)", lambda s: s.get("ego_v")),
        ("Steering Command [-1, 1]", lambda s: s.get("steer")),
    ]
    width, height = 1040, 640
    pw, ph = 420, 220
    origins = [(70, 55), (570, 55), (70, 355), (570, 355)]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
    ]
    # Legend at top
    lx = 70
    for idx, r in enumerate(runs):
        color = palette[idx % len(palette)]
        svg.append(f'<line x1="{lx}" y1="22" x2="{lx+24}" y2="22" stroke="{color}" stroke-width="2.5"/>')
        svg.append(
            f'<text x="{lx+30}" y="26" font-family="sans-serif" font-size="13" fill="#222">{r["label"]}</text>'
        )
        lx += 220

    for (title, getter), (ox, oy) in zip(panels, origins):
        series = []
        t_max = 1.0
        y_min, y_max = math.inf, -math.inf
        for idx, r in enumerate(runs):
            pts = []
            for s in r["samples"]:
                t = s.get("t_rel")
                y = getter(s)
                if isinstance(t, (int, float)) and isinstance(y, (int, float)) and math.isfinite(y):
                    pts.append((t, y))
                    if t > t_max:
                        t_max = t
                    if y < y_min:
                        y_min = y
                    if y > y_max:
                        y_max = y
            series.append((palette[idx % len(palette)], pts))
        if not math.isfinite(y_min) or not math.isfinite(y_max):
            y_min, y_max = -1.0, 1.0
        elif abs(y_max - y_min) < 1e-6:
            y_min -= 0.5
            y_max += 0.5
        else:
            pad = 0.08 * (y_max - y_min)
            y_min -= pad
            y_max += pad

        svg.append(
            f'<text x="{ox + pw/2}" y="{oy - 10}" text-anchor="middle" font-family="sans-serif" font-size="14" font-weight="bold" fill="#222">{title}</text>'
        )
        svg.append(
            f'<rect x="{ox}" y="{oy}" width="{pw}" height="{ph}" fill="#fafafa" stroke="#ccc" stroke-width="1"/>'
        )
        # Zero or mid horizontal grid lines
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            gy = oy + ph * frac
            val = y_max - frac * (y_max - y_min)
            svg.append(f'<line x1="{ox}" y1="{gy:.1f}" x2="{ox+pw}" y2="{gy:.1f}" stroke="#e5e5e5" stroke-width="1"/>')
            svg.append(
                f'<text x="{ox-6}" y="{gy+4:.1f}" text-anchor="end" font-family="monospace" font-size="10" fill="#555">{val:.2f}</text>'
            )
        for frac in (0.0, 0.5, 1.0):
            gx = ox + pw * frac
            tval = frac * t_max
            svg.append(
                f'<text x="{gx:.1f}" y="{oy+ph+15}" text-anchor="middle" font-family="monospace" font-size="10" fill="#555">{tval:.1f}s</text>'
            )

        for color, pts in series:
            if not pts:
                continue
            coords = []
            for t, y in pts:
                px = ox + (t / t_max) * pw
                py = oy + ((y_max - y) / (y_max - y_min)) * ph
                coords.append(f"{px:.1f},{py:.1f}")
            svg.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1.6" points="{" ".join(coords)}"/>'
            )

    svg.append("</svg>\n")
    with open(out_svg, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    return True


def main():
    parser = argparse.ArgumentParser(description="Compare PnC evaluation runs")
    parser.add_argument("run_dirs", nargs="+", help="Two or more run directories containing summary.json")
    parser.add_argument("--out-dir", default=None, help="Directory to save comparison.md and comparison.png")
    args = parser.parse_args()

    runs = [load_run(d) for d in args.run_dirs]
    md = build_markdown(runs)
    print(md)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.run_dirs[0]))
    os.makedirs(out_dir, exist_ok=True)
    md_path = os.path.join(out_dir, "comparison.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md)

    svg_path = os.path.join(out_dir, "comparison.svg")
    save_svg_plots(runs, svg_path)
    png_path = os.path.join(out_dir, "comparison.png")
    plotted = save_plots(runs, png_path)
    print(f"Saved Markdown table: {md_path}")
    print(f"Saved SVG comparison plot: {svg_path}")
    if plotted:
        print(f"Saved PNG comparison plot: {png_path}")


if __name__ == "__main__":
    main()
