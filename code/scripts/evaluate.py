#!/usr/bin/env python3
"""Оценка траекторий относительно эталона EuRoC.

Берутся файлы *.tum из results/<SEQ>/ и (с --runs) из results/<SEQ>/runs/,
где повторные запуски называются <метод>_run<N>.tum.

Стандартные метрики (по всей траектории, совмещение SE(3)):
    ATE RMSE, максимальная ошибка, относительный дрейф на отрезках 10 м.

Метрика потери GNSS (--gnss-loss) считается только по траекториям, выданным
системой в момент кадра (см. is_causal): для моментов t0 через каждые --t0-step секунд
траектория совмещается с эталоном только по окну [t0 - W, t0] (то, что было
известно, пока работал GNSS), и дальше ошибка положения измеряется без
повторного совмещения. Результат -- зависимость ошибки от времени и от пути
после t0, усреднённая по всем t0.

    scripts/evaluate.py MH_01_easy V1_02_medium
    scripts/evaluate.py MH_01_easy --gnss-loss --runs
"""
import argparse
import copy
import csv
import re
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core import metrics, sync
from evo.core.units import Unit
from evo.tools import file_interface

HERE = Path(__file__).resolve().parent.parent
RPE_DELTA_M = 10.0
# запуск считается разошедшимся, если ATE на самом длинном участке больше порога
DIVERGENCE_ATE_M = 1.0
TIME_HORIZONS_S = (10, 30, 60)
DIST_HORIZONS_M = (10, 25, 50)


# ---------- загрузка ----------

def load_pair(gt_path, est_path):
    ref = file_interface.read_euroc_csv_trajectory(str(gt_path))
    est = file_interface.read_tum_trajectory_file(str(est_path))
    return sync.associate_trajectories(ref, est, max_diff=0.01)


def method_name(stem):
    """orbslam3_mono_inertial_run3 -> orbslam3_mono_inertial."""
    return re.sub(r"_run\d+$", "", stem)


def is_causal(method):
    """Траектория выдана системой в момент кадра, без последующих уточнений.

    Итоговые траектории ORB-SLAM3 и файла петель VINS сохраняются в конце
    прогона, после замыканий петель и глобальной оптимизации, то есть каждая
    поза в них уточнена с учётом будущих кадров. Для моделирования потери GNSS
    они не годятся: используются их онлайн-версии (*_online).
    """
    if method.endswith("_online"):
        return True
    if method.startswith("orbslam3") or method.endswith("_loop"):
        return False
    return True  # openvins_*, vins_* (VIO) пишутся по ходу работы


MAX_GAP_S = 0.25
MAX_SPEED_MPS = 5.0  # EuRoC: скорость аппарата не превышает ~2.5 м/с


def segments(ts, pos):
    """Непрерывные участки онлайн-траектории. Граница участка:
    - пауза: система не выдавала позу (потеря трекинга, сброс, новая карта);
    - скачок позы с невозможной скоростью: система перевела карту в другую
      систему координат (инерциальная инициализация, слияние карт).
    После границы связь оценки с прежней системой координат потеряна."""
    dt = np.diff(ts)
    speed = np.linalg.norm(np.diff(pos, axis=0), axis=1) / np.maximum(dt, 1e-6)
    breaks = np.where((dt > MAX_GAP_S) | (speed > MAX_SPEED_MPS))[0] + 1
    return np.split(np.arange(len(ts)), breaks)


def longest_segment(ref, est):
    seg = max(segments(est.timestamps, est.positions_xyz), key=len)
    ref, est = copy.deepcopy(ref), copy.deepcopy(est)
    ref.reduce_to_ids(seg)
    est.reduce_to_ids(seg)
    return ref, est


# ---------- стандартные метрики ----------

def standard_metrics(ref, est):
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
    }, est


# ---------- метрика потери GNSS ----------

def align_window(ref, est, mask):
    """SE(3), переводящее оценку в систему эталона по позам внутри окна.

    Поворот -- хордовое среднее R_ref_i R_est_i^T по окну: в отличие от
    совмещения только по положениям, оно определено и при зависании или
    прямолинейном движении внутри окна (автопилот знает ориентацию и курс,
    пока работает GNSS). Сдвиг -- среднее p_ref - R p_est.
    """
    R_ref = np.array(ref.poses_se3)[mask][:, :3, :3]
    R_est = np.array(est.poses_se3)[mask][:, :3, :3]
    M = np.einsum("nij,nkj->ik", R_ref, R_est)  # сумма R_ref R_est^T
    U, _, Vt = np.linalg.svd(M)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    t = (ref.positions_xyz[mask] - est.positions_xyz[mask] @ R.T).mean(axis=0)
    return R, t


def gnss_loss_samples(ref, est, window_s, t0_step_s):
    """Для каждого t0 -- ошибки после t0 как функция времени и пройденного пути."""
    ts = ref.timestamps
    p_ref = ref.positions_xyz
    path = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p_ref, axis=0), axis=1))]

    seg_id = np.zeros(len(ts), dtype=int)
    for k, ids in enumerate(segments(ts, est.positions_xyz)):
        seg_id[ids] = k

    samples = []  # (t0, dt, dist, err)
    for t0 in np.arange(ts[0] + window_s, ts[-1], t0_step_s):
        i0 = np.searchsorted(ts, t0, side="right") - 1
        same = seg_id == seg_id[i0]
        win = same & (ts >= t0 - window_s) & (ts <= t0)
        # окно должно целиком лежать на одном участке: иначе связь систем
        # координат до t0 определена по позам из другой карты
        if win.sum() < 5 or ts[win][0] - (t0 - window_s) > 0.5:
            continue
        R, t = align_window(ref, est, win)
        # после разрыва (сброс, новая карта) навигация по этой оценке потеряна
        after = same & (ts > t0)
        if not after.any():
            continue
        p_est = est.positions_xyz[after] @ R.T + t
        err = np.linalg.norm(p_est - p_ref[after], axis=1)
        dt = ts[after] - t0
        dist = path[after] - path[i0]
        samples.append((t0, dt, dist, err))
    return samples


def aggregate_curve(samples, axis, bin_size):
    """Средняя, медианная ошибка и 90-й перцентиль по корзинам горизонта."""
    bins = defaultdict(list)
    for _, dt, dist, err in samples:
        x = dt if axis == "time" else dist
        for b, e in zip((x // bin_size).astype(int), err):
            bins[b].append(e)
    rows = []
    for b in sorted(bins):
        v = np.array(bins[b])
        rows.append({"axis": axis, "horizon": (b + 0.5) * bin_size, "n": len(v),
                     "mean_m": v.mean(), "median_m": np.median(v), "p90_m": np.percentile(v, 90)})
    return rows


def error_at(samples, axis, horizon):
    """Медиана по t0 ошибки в момент достижения горизонта (только t0, где он достигнут)."""
    vals = []
    for _, dt, dist, err in samples:
        x = dt if axis == "time" else dist
        k = np.searchsorted(x, horizon)
        if k < len(x):
            vals.append(err[k])
    return (float(np.median(vals)), len(vals)) if vals else (float("nan"), 0)


# ---------- графики ----------

def plot_xy(ref, ests, path, title):
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(ref.positions_xyz[:, 0], ref.positions_xyz[:, 1], "k--", lw=1.2, label="эталон")
    for label, est in ests:
        ax.plot(est.positions_xyz[:, 0], est.positions_xyz[:, 1], lw=1, label=label)
    ax.set_xlabel("x, м")
    ax.set_ylabel("y, м")
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_gnss_curves(curves, path, title):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, axis, xlabel in ((axes[0], "time", "время после потери GNSS, с"),
                             (axes[1], "dist", "путь после потери GNSS, м")):
        for method, rows in curves.items():
            rows = [r for r in rows if r["axis"] == axis and r["n"] >= 3]
            if not rows:
                continue
            x = [r["horizon"] for r in rows]
            line, = ax.plot(x, [r["median_m"] for r in rows], lw=1.2, label=method)
            ax.fill_between(x, [r["median_m"] for r in rows], [r["p90_m"] for r in rows],
                            color=line.get_color(), alpha=0.15)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("ошибка положения, м")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.suptitle(f"{title}: медиана и 90-й перцентиль по моментам потери GNSS", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ---------- основной цикл ----------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sequences", nargs="+")
    ap.add_argument("--runs", action="store_true", help="учитывать повторные запуски из results/<SEQ>/runs/")
    ap.add_argument("--gnss-loss", action="store_true", help="считать метрику потери GNSS")
    ap.add_argument("--window", type=float, default=10.0, help="окно совмещения до t0, с")
    ap.add_argument("--t0-step", type=float, default=5.0, help="шаг перебора t0, с")
    args = ap.parse_args()

    rows, gnss_rows = [], []
    for seq in args.sequences:
        gt = HERE / f"data/euroc/{seq}/mav0/state_groundtruth_estimate0/data.csv"
        res = HERE / f"results/{seq}"
        gt_ts = file_interface.read_euroc_csv_trajectory(str(gt)).timestamps
        gt_duration = gt_ts[-1] - gt_ts[0]
        files = sorted(res.glob("*.tum"))
        if args.runs:
            # последний запуск лежит и в results/SEQ, и в runs/: не считаем его дважды
            run_files = sorted((res / "runs").glob("*.tum"))
            in_runs = {method_name(f.stem) for f in run_files}
            files = [f for f in files if f.stem not in in_runs] + run_files

        xy, curves, ref_for_plot = [], defaultdict(list), None
        samples_by_method = defaultdict(list)
        for est_path in files:
            run = est_path.stem
            method = method_name(run)
            ref, est = load_pair(gt, est_path)
            # онлайн-траектории могут содержать сбросы: стандартные метрики по ним
            # считаются по самому длинному непрерывному участку. Итоговые файлы
            # (петли VINS -- только ключевые кадры) не делятся: паузы в них не сбросы.
            n_seg = len(segments(est.timestamps, est.positions_xyz)) if is_causal(method) else 1
            pair = longest_segment(ref, est) if n_seg > 1 else (ref, est)
            stats, aligned = standard_metrics(*pair)
            stats["segments"] = n_seg
            # доля полёта, покрытая самым длинным непрерывным участком
            stats["coverage"] = stats["duration_s"] / gt_duration
            diverged = stats["ate_rmse_m"] > DIVERGENCE_ATE_M
            rows.append({"sequence": seq, "method": method, "run": run, "diverged": int(diverged), **stats})

            if run == method or run == f"{method}_run1":  # на график -- один запуск на метод
                xy.append((f"{method} ({stats['ate_rmse_m'] * 100:.1f} см)", aligned))
                if ref_for_plot is None or ref.num_poses > ref_for_plot.num_poses:
                    ref_for_plot = ref
            if args.gnss_loss and not diverged and is_causal(method):
                samples_by_method[method] += gnss_loss_samples(ref, est, args.window, args.t0_step)

        if ref_for_plot is not None:
            plot_xy(ref_for_plot, xy, res / "traj_xy.png", seq)

        for method, samples in samples_by_method.items():
            for axis, size in (("time", 1.0), ("dist", 1.0)):
                curves[method] += aggregate_curve(samples, axis, size)
            g = {"sequence": seq, "method": method, "t0_count": len(samples)}
            for h in TIME_HORIZONS_S:
                g[f"err_{h}s_m"], g[f"n_{h}s"] = error_at(samples, "time", h)
            for h in DIST_HORIZONS_M:
                g[f"err_{h}m_m"], g[f"n_{h}m"] = error_at(samples, "dist", h)
            gnss_rows.append(g)
        if curves:
            plot_gnss_curves(curves, res / "gnss_loss.png", seq)
            with open(res / "gnss_loss_curves.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["method", "axis", "horizon", "n", "mean_m", "median_m", "p90_m"])
                w.writeheader()
                for method, rs in curves.items():
                    for r in rs:
                        w.writerow({"method": method, **r})

    write_csv(HERE / "results/summary.csv", rows)
    print_summary(rows, args.runs)
    if gnss_rows:
        write_csv(HERE / "results/gnss_loss_summary.csv", gnss_rows)
        print_gnss(gnss_rows)


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def print_summary(rows, by_method):
    if not by_method:
        print(f"{'последовательность':18s} {'метод':26s} {'поз':>5s} {'путь,м':>7s} {'ATE,см':>7s} {'max,см':>7s} {'дрейф,%':>8s} {'участков':>8s} {'покрытие':>8s}")
        for r in rows:
            print(f"{r['sequence']:18s} {r['run']:26s} {r['poses']:5d} {r['length_m']:7.1f} "
                  f"{r['ate_rmse_m'] * 100:7.1f} {r['ate_max_m'] * 100:7.1f} {r['drift_10m_pct']:8.2f} {r['segments']:8d} {r['coverage']:8.0%}")
        return
    # по нескольким запускам: среднее и разброс по удачным, доля расхождений
    groups = defaultdict(list)
    for r in rows:
        groups[(r["sequence"], r["method"])].append(r)
    print(f"{'последовательность':18s} {'метод':26s} {'запусков':>8s} {'расх.':>5s} {'ATE,см':>13s} {'дрейф,%':>12s} {'покрытие':>8s}")
    for (seq, method), rs in groups.items():
        ok = [r for r in rs if not r["diverged"]]
        ate = np.array([r["ate_rmse_m"] for r in ok]) * 100
        drift = np.array([r["drift_10m_pct"] for r in ok])
        a = f"{ate.mean():5.1f} ± {ate.std():4.1f}" if len(ok) else "-"
        d = f"{drift.mean():4.2f} ± {drift.std():4.2f}" if len(ok) else "-"
        cov = np.mean([r["coverage"] for r in ok]) if ok else float("nan")
        print(f"{seq:18s} {method:26s} {len(rs):8d} {len(rs) - len(ok):5d} {a:>13s} {d:>12s} {cov:8.0%}")


def print_gnss(rows):
    hdr = " ".join(f"{h:>4d}с" for h in TIME_HORIZONS_S) + " " + " ".join(f"{h:>4d}м" for h in DIST_HORIZONS_M)
    print(f"\nОшибка после потери GNSS, м (медиана по t0)\n{'последовательность':18s} {'метод':26s} {hdr}")
    for r in rows:
        vals = [r[f"err_{h}s_m"] for h in TIME_HORIZONS_S] + [r[f"err_{h}m_m"] for h in DIST_HORIZONS_M]
        print(f"{r['sequence']:18s} {r['method']:26s} " + " ".join(f"{v:5.2f}" for v in vals))


if __name__ == "__main__":
    main()
