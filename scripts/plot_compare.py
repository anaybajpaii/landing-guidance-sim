import argparse
import csv
import os
import sys

import matplotlib.pyplot as plt


def read_summary(path):
    if not os.path.exists(path):
        sys.exit("Cannot find " + path + " (run the simulator with --run first, from the repo root)")
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit("No runs found in " + path)
    return rows


def read_run(path):
    columns = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames:
            columns[name] = []
        for row in reader:
            for name in reader.fieldnames:
                columns[name].append(float(row[name]))
    return columns


def verdict(row):
    if row["landed"] != "1":
        return "TIMEOUT"
    return "SUCCESS" if row["success"] == "1" else "FAIL"


def line_panel(ax, runs, colors, column, title, ylabel):
    for run, color in zip(runs, colors):
        ax.plot(run["t"], run[column], color=color, linewidth=1.6)
    ax.set_title(title)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def bar_panel(ax, rows, colors, values, title, ylabel, limit=None):
    positions = list(range(1, len(rows) + 1))
    bars = ax.bar(positions, values, color=colors)
    for bar, row in zip(bars, rows):
        if verdict(row) != "SUCCESS":
            bar.set_hatch("//")
            bar.set_edgecolor("black")
    if limit is not None:
        ax.axhline(limit, color="black", linestyle="--", linewidth=1)
        ax.text(0.5, limit, " limit", va="bottom", ha="left", fontsize=8,
                transform=ax.get_yaxis_transform())
    ax.set_xticks(positions)
    ax.set_title(title)
    ax.set_xlabel("run")
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.3)


def table_panel(ax, rows, colors):
    ax.axis("off")
    ax.set_title("results")
    cells = []
    for row in rows:
        cells.append([row["run"], verdict(row), "%.2f" % float(row["time"])])
    table = ax.table(cellText=cells, colLabels=["run", "result", "time (s)"],
                     loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 1.4)
    for i, color in enumerate(colors, start=1):
        table[(i, 0)].set_facecolor(color)
        table[(i, 0)].get_text().set_color("white")


def main():
    parser = argparse.ArgumentParser(description="Overlay several lander runs from a summary CSV.")
    parser.add_argument("summary", nargs="?", default="data/summary.csv",
                        help="summary CSV written by lander --run (default data/summary.csv)")
    parser.add_argument("--save", help="save the figure to this file instead of opening a window")
    parser.add_argument("--limit", type=float, default=2.0,
                        help="landing speed limit drawn on the bar chart (default 2.0)")
    args = parser.parse_args()

    rows = read_summary(args.summary)
    runs = []
    for row in rows:
        if not os.path.exists(row["csv_file"]):
            sys.exit("Cannot find " + row["csv_file"] + " (run this from the repo root)")
        runs.append(read_run(row["csv_file"]))

    cmap = plt.get_cmap("tab10")
    colors = [cmap(i % 10) for i in range(len(rows))]

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    line_panel(axes[0][0], runs, colors, "y", "altitude vs time", "altitude (m)")
    line_panel(axes[0][1], runs, colors, "vy", "vertical velocity vs time", "vertical velocity (m/s)")
    line_panel(axes[0][2], runs, colors, "thrust", "thrust vs time", "thrust (N)")

    fuel = [float(r["fuel_used"]) for r in rows]
    speeds = [float(r["landing_speed"]) if r["landed"] == "1" else 0.0 for r in rows]
    bar_panel(axes[1][0], rows, colors, fuel, "fuel used", "fuel (kg)")
    bar_panel(axes[1][1], rows, colors, speeds, "landing speed (hatched = not a success)",
              "speed (m/s)", args.limit)
    table_panel(axes[1][2], rows, colors)

    handles = [plt.Line2D([0], [0], color=c, linewidth=3) for c in colors]
    names = [str(i + 1) + ": " + r["label"] for i, r in enumerate(rows)]
    fig.legend(handles, names, loc="lower center", ncol=min(len(rows), 3), frameon=False)
    fig.tight_layout(rect=(0, 0.08, 1, 1))

    if args.save:
        fig.savefig(args.save, dpi=150)
        print("saved " + args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()