#!/usr/bin/env python3
"""Рисунок для отчёта: траектории всех методов на нескольких последовательностях (вид сверху).

    scripts/plot_trajectories.py --out ../graphics/euroc_traj.png MH_01_easy MH_03_medium ...
"""
import argparse
import copy
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.core import sync
from evo.tools import file_interface

HERE = Path(__file__).resolve().parent.parent
# палитра проверена на различимость при дальтонизме; стиль линии -- второй признак
METHODS = {
    "orbslam3_mono_inertial": ("ORB-SLAM3", "#E69F00", "-"),
    "vins_mono_imu_loop": ("VINS-Fusion + петли", "#B2182B", "--"),
    "vins_mono_imu": ("VINS-Fusion (VIO)", "#009E73", "-."),
    "openvins_mono": ("OpenVINS", "#0072B2", ":"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sequences", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    n = len(args.sequences)
    cols = 3
    rows = (n + 1 + cols - 1) // cols  # +1 ячейка под легенду
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 3.8 * rows))
    axes = axes.ravel()
    for ax, seq in zip(axes, args.sequences):
        gt = file_interface.read_euroc_csv_trajectory(
            str(HERE / f"data/euroc/{seq}/mav0/state_groundtruth_estimate0/data.csv"))
        ax.plot(gt.positions_xyz[:, 0], gt.positions_xyz[:, 1], color="0.25", lw=1.4, alpha=0.5)
        for stem, (label, color, ls) in METHODS.items():
            path = HERE / f"results/{seq}/{stem}.tum"
            if not path.exists():
                continue
            est = file_interface.read_tum_trajectory_file(str(path))
            ref, est = sync.associate_trajectories(copy.deepcopy(gt), est, max_diff=0.01)
            est.align(ref, correct_scale=False)
            ax.plot(est.positions_xyz[:, 0], est.positions_xyz[:, 1], color=color, ls=ls, lw=0.9)
        ax.set_title(seq, fontsize=10)
        ax.set_aspect("equal", adjustable="datalim")
        ax.tick_params(labelsize=7)
        ax.set_xlabel("x, м", fontsize=8)
        ax.set_ylabel("y, м", fontsize=8)

    leg = axes[n]
    leg.axis("off")
    leg.plot([], [], color="0.25", lw=1.4, alpha=0.5, label="эталон")
    for label, color, ls in METHODS.values():
        leg.plot([], [], color=color, ls=ls, lw=1.2, label=label)
    leg.legend(loc="center", fontsize=10, frameon=False, handlelength=3)
    for ax in axes[n + 1:]:
        ax.axis("off")

    fig.tight_layout()
    fig.savefig(args.out, dpi=200)


if __name__ == "__main__":
    main()
