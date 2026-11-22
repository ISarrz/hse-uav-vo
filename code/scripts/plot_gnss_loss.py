#!/usr/bin/env python3
"""Рисунок для отчёта: ошибка положения после потери GNSS на нескольких последовательностях.

Читает results/<SEQ>/gnss_loss_curves.csv (создаёт evaluate.py --gnss-loss).

    scripts/plot_gnss_loss.py --out ../graphics/euroc_gnss_loss.png MH_01_easy ...
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent.parent
# палитра проверена на различимость при дальтонизме; стиль линии -- второй признак
METHODS = {
    "orbslam3_mono_inertial_online": ("ORB-SLAM3", "#E69F00", "-"),
    "vins_mono_imu_loop_online": ("VINS-Fusion + петли", "#B2182B", "--"),
    "vins_mono_imu": ("VINS-Fusion (VIO)", "#009E73", "-."),
    "openvins_mono": ("OpenVINS", "#0072B2", ":"),
}
MIN_SAMPLES = 60  # отбрасываем горизонты, достигнутые лишь из нескольких t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sequences", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--axis", choices=["time", "dist"], default="time")
    ap.add_argument("--max", type=float, default=60)
    args = ap.parse_args()

    n = len(args.sequences)
    cols = 3
    rows = (n + 1 + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.2 * rows))
    axes = axes.ravel()
    for ax, seq in zip(axes, args.sequences):
        curves = defaultdict(list)
        with open(HERE / f"results/{seq}/gnss_loss_curves.csv") as f:
            for r in csv.DictReader(f):
                h = float(r["horizon"])
                if r["axis"] == args.axis and h <= args.max and int(r["n"]) >= MIN_SAMPLES:
                    curves[r["method"]].append((h, float(r["median_m"]) * 100))
        for method, (label, color, ls) in METHODS.items():
            pts = curves.get(method)
            if pts:
                ax.plot(*zip(*pts), color=color, ls=ls, lw=1.6)
        ax.set_title(seq, fontsize=10)
        ax.set_xlim(0, args.max)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.25, lw=0.5)
        ax.tick_params(labelsize=7)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        ax.set_xlabel("время после потери GNSS, с" if args.axis == "time" else "путь после потери GNSS, м", fontsize=8)
        ax.set_ylabel("ошибка положения, см", fontsize=8)

    leg = axes[n]
    leg.axis("off")
    for label, color, ls in METHODS.values():
        leg.plot([], [], color=color, ls=ls, lw=1.6, label=label)
    leg.legend(loc="center", fontsize=10, frameon=False, handlelength=3)
    for ax in axes[n + 1:]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(args.out, dpi=200)


if __name__ == "__main__":
    main()
