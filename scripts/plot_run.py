import argparse

import matplotlib.pyplot as plt
import numpy as np


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def plot_run(data):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    t = data["t"]

    axes[0, 0].plot(t, data["y"])
    axes[0, 0].set_title("Altitude")
    axes[0, 0].set_xlabel("time (s)")
    axes[0, 0].set_ylabel("altitude (m)")

    axes[0, 1].plot(t, data["vy"], label="vertical")
    axes[0, 1].plot(t, data["vx"], label="horizontal")
    axes[0, 1].set_title("Velocity")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("velocity (m/s)")
    axes[0, 1].legend()

    axes[1, 0].plot(t, data["thrust"])
    axes[1, 0].set_title("Thrust")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("thrust (N)")

    axes[1, 1].plot(t, data["mass"])
    axes[1, 1].set_title("Mass")
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].set_ylabel("mass (kg)")

    for ax in axes.flat:
        ax.grid(True)

    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot one landing run.")
    parser.add_argument("csv", nargs="?", default="data/run.csv")
    parser.add_argument("--save", help="write a PNG to this path instead of opening a window")
    args = parser.parse_args()

    data = load(args.csv)
    fig = plot_run(data)

    if args.save:
        fig.savefig(args.save, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()