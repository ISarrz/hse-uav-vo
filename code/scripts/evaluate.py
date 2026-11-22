#!/usr/bin/env python3
"""Оценка траекторий относительно эталона EuRoC.

Для каждой последовательности в results/<SEQ>/ берутся все файлы *.tum,
совмещаются с эталоном преобразованием SE(3) (для VIO масштаб наблюдаем)
и считаются ATE RMSE и относительный дрейф на отрезках 10 м.
Результаты пишутся в results/summary.csv, графики -- в results/<SEQ>/traj_xy.png.

    scripts/evaluate.py MH_01_easy V1_02_medium
"""
import argparse
import copy
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.core import metrics, sync
from evo.core.units import Unit
from evo.tools import file_interface

HERE = Path(__file__).resolve().parent.parent
RPE_DELTA_M = 10.0


def evaluate(gt_path, est_path):
    ref = file_interface.read_euroc_csv_trajectory(str(gt_path))
    est = file_interface.read_tum_trajectory_file(str(est_path))
    ref, est = sync.associate_trajectories(ref, est, max_diff=0.01)
    est = copy.deepcopy(est)
    est.align(ref, correct_scale=False)

    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref, est))

    rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=RPE_DELTA_M,
                      delta_unit=Unit.meters, all_pairs=True)
    try:
        rpe.process_data((ref, est))
        drift = rpe.get_statistic(metrics.StatisticsType.mean) / RPE_DELTA_M * 100
    except Exception:
        drift = float("nan")  # траектория короче 10 м

    return {
        "poses": est.num_poses,
        "duration_s": est.timestamps[-1] - est.timestamps[0],
        "length_m": ref.path_length,
        "ate_rmse_m": ape.get_statistic(metrics.StatisticsType.rmse),
        "ate_max_m": ape.get_statistic(metrics.StatisticsType.max),
        "drift_10m_pct": drift,
    }, ref, est


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sequences", nargs="+")
    args = ap.parse_args()

    rows = []
    for seq in args.sequences:
        gt = HERE / f"data/euroc/{seq}/mav0/state_groundtruth_estimate0/data.csv"
        res = HERE / f"results/{seq}"
        fig, ax = plt.subplots(figsize=(6, 5))
        ref_drawn = False
        for est_path in sorted(res.glob("*.tum")):
            stats, ref, est = evaluate(gt, est_path)
            rows.append({"sequence": seq, "method": est_path.stem, **stats})
            if not ref_drawn:
                ax.plot(ref.positions_xyz[:, 0], ref.positions_xyz[:, 1], "k--", lw=1.2, label="эталон")
                ref_drawn = True
            ax.plot(est.positions_xyz[:, 0], est.positions_xyz[:, 1], lw=1,
                    label=f"{est_path.stem} ({stats['ate_rmse_m'] * 100:.1f} см)")
        ax.set_xlabel("x, м")
        ax.set_ylabel("y, м")
        ax.set_aspect("equal")
        ax.set_title(seq)
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(res / "traj_xy.png", dpi=150)
        plt.close(fig)

    with open(HERE / "results/summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"{'последовательность':18s} {'метод':26s} {'поз':>5s} {'путь,м':>7s} {'ATE,см':>7s} {'max,см':>7s} {'дрейф,%':>8s}")
    for r in rows:
        print(f"{r['sequence']:18s} {r['method']:26s} {r['poses']:5d} {r['length_m']:7.1f} "
              f"{r['ate_rmse_m'] * 100:7.1f} {r['ate_max_m'] * 100:7.1f} {r['drift_10m_pct']:8.2f}")


if __name__ == "__main__":
    main()
