#!/usr/bin/env python3
"""
analyze_lidar.py

Analysis for the Phase 0 TFmini-S over-seawater bench test. Reads the raw
CSV written by pc_csv_capture.py and produces:

  - a per-burst summary table (condition, angle, rig height, sample count,
    valid fraction, distance/strength stats, inter-sample timing)
  - a boresight fit from the "wall" condition sweep
  - a surface-vs-bottom verdict from calm_shallow vs calm_deep at matched
    rig height
  - four diagnostic plots, saved as PNG

Nothing is dropped while reading. Every row in the CSV becomes one record
in memory, with a validity flag and, if applicable, a parse-error flag.
Filtering only happens downstream, when a specific statistic is computed
(e.g. distance mean uses valid samples only). This is deliberate: capture
and parsing must be lossless, filtering belongs to analysis.

Usage:
    python analyze_lidar.py path/to/log.csv
    python analyze_lidar.py path/to/log.csv --outdir path/to/plots
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import optimize, stats

FIELDNAMES = ["wall_clock", "burst_id", "t_ms", "distance_cm", "strength",
              "temp_c", "angle_deg", "rig_height_m", "condition"]

STRENGTH_MIN = 100
STRENGTH_SATURATED = 65535
DISTANCE_ZERO = 0
DISTANCE_MAX = 65535

BIMODALITY_THRESHOLD = 5.0 / 9.0   # Pfister et al. / SAS rule of thumb


# ----------------------------------------------------------------------
# Loading: never drop a row, only flag it
# ----------------------------------------------------------------------

def _parse_int(s):
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return None


def _parse_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return math.nan


def load_records(path):
    """Read the raw capture CSV into a list of per-row dicts.

    Column position is trusted over the header text (a hand-edited or
    corrupted header should not stop this from reading the data). Rows
    with the wrong column count are padded/truncated rather than
    rejected. Rows whose burst_id will not parse still get an entry:
    they are put into a unique single-row synthetic burst so they show
    up in the summary instead of vanishing.
    """
    records = []
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        try:
            next(reader)   # header, not otherwise used
        except StopIteration:
            return records

        for row_idx, row in enumerate(reader):
            if not row:
                continue
            malformed_columns = len(row) != len(FIELDNAMES)
            padded = (row + [""] * len(FIELDNAMES))[:len(FIELDNAMES)]
            (wall_clock, burst_raw, t_raw, dist_raw, str_raw,
             temp_raw, ang_raw, h_raw, cond_raw) = padded

            burst_id = _parse_int(burst_raw)
            t_ms = _parse_int(t_raw)
            distance_cm = _parse_int(dist_raw)
            strength = _parse_int(str_raw)
            temp_c = _parse_float(temp_raw)
            angle_deg = _parse_float(ang_raw)
            rig_height_m = _parse_float(h_raw)
            condition_raw = cond_raw
            condition = condition_raw.strip().lower()

            parse_error = (burst_id is None or t_ms is None
                          or distance_cm is None or strength is None)

            low_strength = strength is not None and strength < STRENGTH_MIN
            saturated = strength is not None and strength == STRENGTH_SATURATED
            zero_distance = distance_cm is not None and distance_cm == DISTANCE_ZERO
            max_distance = distance_cm is not None and distance_cm == DISTANCE_MAX
            invalid = parse_error or low_strength or saturated or zero_distance or max_distance

            records.append(dict(
                row_idx=row_idx,
                malformed_columns=malformed_columns,
                wall_clock=wall_clock,
                burst_id=burst_id if burst_id is not None else f"UNPARSEABLE_row{row_idx}",
                burst_id_raw=burst_raw,
                t_ms=t_ms,
                distance_cm=distance_cm,
                strength=strength,
                temp_c=temp_c,
                angle_deg=angle_deg,
                rig_height_m=rig_height_m,
                condition_raw=condition_raw,
                condition=condition,
                parse_error=parse_error,
                low_strength=low_strength,
                saturated=saturated,
                zero_distance=zero_distance,
                max_distance=max_distance,
                valid=not invalid,
            ))
    return records


def group_bursts(records):
    bursts = defaultdict(list)
    for r in records:
        bursts[r["burst_id"]].append(r)
    return bursts


# ----------------------------------------------------------------------
# Per-burst summary
# ----------------------------------------------------------------------

def summarize_burst(burst_id, recs):
    n = len(recs)
    conditions = sorted(set(r["condition_raw"] for r in recs if r["condition_raw"]))
    condition_label = "/".join(conditions) if conditions else "(missing)"

    angles = [r["angle_deg"] for r in recs if not math.isnan(r["angle_deg"])]
    angle = angles[0] if angles else math.nan
    heights = [r["rig_height_m"] for r in recs if not math.isnan(r["rig_height_m"])]
    height = heights[0] if heights else math.nan

    n_parse_error = sum(r["parse_error"] for r in recs)
    n_low_strength = sum(r["low_strength"] for r in recs)
    n_saturated = sum(r["saturated"] for r in recs)
    n_zero_distance = sum(r["zero_distance"] for r in recs)
    n_max_distance = sum(r["max_distance"] for r in recs)
    n_valid = sum(r["valid"] for r in recs)
    valid_fraction = n_valid / n if n else math.nan

    dist_valid = np.array([r["distance_cm"] for r in recs if r["valid"]], dtype=float)
    mean_dist = float(np.mean(dist_valid)) if dist_valid.size else math.nan
    std_dist = float(np.std(dist_valid, ddof=1)) if dist_valid.size > 1 else math.nan

    # Strength is diagnostic on its own (it is what separates dropout modes),
    # so its statistics use every sample that parsed, valid or not.
    strengths_all = np.array([r["strength"] for r in recs if r["strength"] is not None], dtype=float)
    mean_strength = float(np.mean(strengths_all)) if strengths_all.size else math.nan
    median_strength = float(np.median(strengths_all)) if strengths_all.size else math.nan

    t_ms_ok = sorted(r["t_ms"] for r in recs if r["t_ms"] is not None)
    if len(t_ms_ok) >= 2:
        gaps = np.diff(np.array(t_ms_ok, dtype=float))
        median_gap = float(np.median(gaps))
        max_gap = float(np.max(gaps))
        span_ms = float(t_ms_ok[-1] - t_ms_ok[0])
    else:
        median_gap = max_gap = span_ms = math.nan

    return dict(
        burst_id=burst_id, condition=condition_label, angle_deg=angle,
        rig_height_m=height, n=n, valid_fraction=valid_fraction,
        mean_distance_cm=mean_dist, std_distance_cm=std_dist,
        mean_strength=mean_strength, median_strength=median_strength,
        median_gap_ms=median_gap, max_gap_ms=max_gap, span_ms=span_ms,
        n_parse_error=n_parse_error, n_low_strength=n_low_strength,
        n_saturated=n_saturated, n_zero_distance=n_zero_distance,
        n_max_distance=n_max_distance,
    )


def print_summary_table(summaries):
    print("Per-burst summary")
    print("=" * 118)
    header = (f"{'burst':>8}  {'condition':<14} {'angle':>6} {'height':>7} "
              f"{'n':>5} {'valid%':>7} {'dist_mean':>10} {'dist_std':>9} "
              f"{'str_mean':>9} {'str_med':>8} {'gap_med':>8} {'gap_max':>8}")
    print(header)
    print("-" * 118)
    for s in summaries:
        print(f"{str(s['burst_id']):>8}  {s['condition']:<14} "
              f"{s['angle_deg']:>6.1f} {s['rig_height_m']:>7.2f} "
              f"{s['n']:>5d} {100*s['valid_fraction']:>6.1f}% "
              f"{s['mean_distance_cm']:>10.2f} {s['std_distance_cm']:>9.2f} "
              f"{s['mean_strength']:>9.0f} {s['median_strength']:>8.0f} "
              f"{s['median_gap_ms']:>8.1f} {s['max_gap_ms']:>8.1f}")
    print("-" * 118)
    print("Failure-mode counts per burst (not mutually exclusive, a row can match more than one):")
    fail_header = (f"{'burst':>8}  {'parse_err':>9} {'low_str':>8} "
                   f"{'saturated':>9} {'dist=0':>7} {'dist=max':>8}")
    print(fail_header)
    for s in summaries:
        print(f"{str(s['burst_id']):>8}  {s['n_parse_error']:>9d} {s['n_low_strength']:>8d} "
              f"{s['n_saturated']:>9d} {s['n_zero_distance']:>7d} {s['n_max_distance']:>8d}")
    print("=" * 118)


# ----------------------------------------------------------------------
# Boresight fit: d(theta) = d_perp / cos(theta - theta0)
# ----------------------------------------------------------------------

def fit_boresight(records):
    wall_recs = [r for r in records
                 if r["condition"] == "wall" and r["valid"]
                 and not math.isnan(r["angle_deg"])]
    if not wall_recs:
        return {"ok": False, "reason": "no wall-condition samples in this CSV"}

    theta = np.radians(np.array([r["angle_deg"] for r in wall_recs], dtype=float))
    d = np.array([r["distance_cm"] for r in wall_recs], dtype=float)

    if len(np.unique(theta)) < 2:
        return {"ok": False, "reason": "wall sweep has fewer than two distinct angles", "n": len(wall_recs)}

    def model(theta, d_perp, theta0):
        return d_perp / np.cos(theta - theta0)

    theta0_guess = float(theta[np.argmin(d)])
    d_perp_guess = float(np.min(d))
    bounds = ([1e-6, -np.pi / 2 + 1e-6], [np.inf, np.pi / 2 - 1e-6])

    try:
        popt, pcov = optimize.curve_fit(
            model, theta, d, p0=[d_perp_guess, theta0_guess], bounds=bounds)
    except (RuntimeError, ValueError) as exc:
        return {"ok": False, "reason": f"fit did not converge ({exc})", "n": len(wall_recs)}

    if not np.all(np.isfinite(popt)):
        return {"ok": False, "reason": "fit produced non-finite parameters", "n": len(wall_recs)}

    d_perp, theta0 = popt
    if pcov is not None and np.all(np.isfinite(pcov)):
        perr = np.sqrt(np.diag(pcov))
    else:
        perr = [math.nan, math.nan]

    residuals = d - model(theta, *popt)
    rms_residual = float(np.sqrt(np.mean(residuals ** 2)))

    return {
        "ok": True, "n": len(wall_recs),
        "d_perp_cm": float(d_perp),
        "theta0_deg": math.degrees(theta0),
        "theta0_unc_deg": math.degrees(perr[1]),
        "rms_residual_cm": rms_residual,
    }


def print_boresight(fit):
    print("Boresight fit (wall condition, d(theta) = d_perp / cos(theta - theta0))")
    print("-" * 118)
    if not fit["ok"]:
        print(f"  Not computed: {fit['reason']}.")
        return
    print(f"  n = {fit['n']} samples")
    print(f"  d_perp   = {fit['d_perp_cm']:.2f} cm")
    print(f"  theta_0  = {fit['theta0_deg']:+.3f} deg  (uncertainty {fit['theta0_unc_deg']:.3f} deg)")
    print(f"  residual = {fit['rms_residual_cm']:.2f} cm RMS")
    print(f"  All angles in this dataset should be read relative to theta_0, "
          f"not to the printed protractor mark.")


# ----------------------------------------------------------------------
# Surface vs bottom verdict
# ----------------------------------------------------------------------

def bimodality_coefficient(x):
    """Pfister et al. sample bimodality coefficient. BC > 5/9 is the usual
    rule-of-thumb flag for a bimodal distribution. NaN if too few points
    to estimate skew/kurtosis meaningfully."""
    x = np.asarray(x, dtype=float)
    n = x.size
    if n < 4:
        return math.nan
    skew = stats.skew(x, bias=False)
    kurt = stats.kurtosis(x, fisher=True, bias=False)
    denom = kurt + 3 * (n - 1) ** 2 / ((n - 2) * (n - 3))
    if denom == 0:
        return math.nan
    return float((skew ** 2 + 1) / denom)


def surface_vs_bottom_verdict(records):
    shallow = [r for r in records if r["condition"] == "calm_shallow" and r["valid"]]
    deep = [r for r in records if r["condition"] == "calm_deep" and r["valid"]]

    missing = []
    if not shallow:
        missing.append("calm_shallow")
    if not deep:
        missing.append("calm_deep")
    if missing:
        return {"ok": False, "reason": f"no valid samples for: {', '.join(missing)}"}

    by_height_s = defaultdict(list)
    for r in shallow:
        if not math.isnan(r["rig_height_m"]):
            by_height_s[round(r["rig_height_m"], 2)].append(r)
    by_height_d = defaultdict(list)
    for r in deep:
        if not math.isnan(r["rig_height_m"]):
            by_height_d[round(r["rig_height_m"], 2)].append(r)

    common_heights = sorted(set(by_height_s) & set(by_height_d))
    if not common_heights:
        return {"ok": False, "reason": "calm_shallow and calm_deep never share a matching rig height"}
    h = common_heights[0]
    s_recs, d_recs = by_height_s[h], by_height_d[h]

    s_d = np.array([r["distance_cm"] for r in s_recs], dtype=float)
    d_d = np.array([r["distance_cm"] for r in d_recs], dtype=float)
    mean_s, mean_d = float(np.mean(s_d)), float(np.mean(d_d))
    std_s = float(np.std(s_d, ddof=1)) if s_d.size > 1 else 0.0
    std_d = float(np.std(d_d, ddof=1)) if d_d.size > 1 else 0.0
    diff = mean_d - mean_s
    pooled_se = math.sqrt((std_s ** 2) / s_d.size + (std_d ** 2) / d_d.size)
    if pooled_se > 0:
        t_stat, p_value = stats.ttest_ind(d_d, s_d, equal_var=False)
    else:
        t_stat, p_value = math.nan, math.nan
    significant = (not math.isnan(p_value)) and p_value < 0.05

    rig_height_cm = h * 100.0
    tol_cm = max(3.0, 2.0 * max(std_s, std_d))
    shallow_is_surface = abs(mean_s - rig_height_cm) <= tol_cm
    deep_is_surface = abs(mean_d - rig_height_cm) <= tol_cm

    bc_s = bimodality_coefficient(s_d)
    bc_d = bimodality_coefficient(d_d)
    distance_bimodal = (not math.isnan(bc_s) and bc_s > BIMODALITY_THRESHOLD) or \
                        (not math.isnan(bc_d) and bc_d > BIMODALITY_THRESHOLD)

    bc_strength = None
    strength_separates = None
    if distance_bimodal:
        combined_strength = np.array(
            [r["strength"] for r in s_recs + d_recs if r["strength"] is not None], dtype=float)
        bc_strength = bimodality_coefficient(combined_strength)
        strength_separates = (not math.isnan(bc_strength)) and bc_strength > BIMODALITY_THRESHOLD

    if distance_bimodal:
        verdict = "mixed"
    elif shallow_is_surface and deep_is_surface and not significant:
        verdict = "surface return"
    elif significant and not (shallow_is_surface and deep_is_surface):
        verdict = "bottom return"
    else:
        verdict = "mixed"

    return {
        "ok": True, "rig_height_m": h, "n_shallow": int(s_d.size), "n_deep": int(d_d.size),
        "mean_shallow_cm": mean_s, "mean_deep_cm": mean_d, "diff_cm": diff,
        "pooled_se_cm": pooled_se, "t_stat": float(t_stat) if not math.isnan(t_stat) else math.nan,
        "p_value": float(p_value) if not math.isnan(p_value) else math.nan,
        "significant": significant, "verdict": verdict,
        "bimodality_shallow": bc_s, "bimodality_deep": bc_d,
        "bimodality_strength": bc_strength, "strength_separates": strength_separates,
    }


def print_verdict(v):
    print("Surface versus bottom verdict")
    print("-" * 118)
    if not v["ok"]:
        print(f"  Not computed: {v['reason']}.")
        return
    print(f"  Matched rig height: {v['rig_height_m']:.2f} m "
          f"(shallow n={v['n_shallow']}, deep n={v['n_deep']})")
    print(f"  mean distance  shallow {v['mean_shallow_cm']:.2f} cm   "
          f"deep {v['mean_deep_cm']:.2f} cm   diff {v['diff_cm']:+.2f} cm")
    print(f"  pooled SE of the difference {v['pooled_se_cm']:.2f} cm   "
          f"Welch t={v['t_stat']:.2f}  p={v['p_value']:.4f}  "
          f"({'significant' if v['significant'] else 'not significant'} at p<0.05)")
    print(f"  distance bimodality coefficient: shallow {v['bimodality_shallow']:.3f}  "
          f"deep {v['bimodality_deep']:.3f}  (flag above {BIMODALITY_THRESHOLD:.3f})")
    if v["bimodality_strength"] is not None:
        print(f"  strength bimodality coefficient (mixed case): {v['bimodality_strength']:.3f}  "
              f"-> strength {'does' if v['strength_separates'] else 'does not'} "
              f"cleanly separate the two return modes")

    sentence = {
        "surface return": (
            f"Verdict: surface return -- at {v['rig_height_m']:.2f} m rig height, "
            f"calm_shallow ({v['mean_shallow_cm']:.1f} cm) and calm_deep "
            f"({v['mean_deep_cm']:.1f} cm) read the same distance regardless of water "
            f"depth, consistent with the LiDAR reflecting off the surface."
        ),
        "bottom return": (
            f"Verdict: bottom return -- at {v['rig_height_m']:.2f} m rig height, "
            f"calm_shallow ({v['mean_shallow_cm']:.1f} cm) and calm_deep "
            f"({v['mean_deep_cm']:.1f} cm) differ by {v['diff_cm']:+.1f} cm in a way "
            f"that tracks water depth, consistent with the beam reaching the bottom."
        ),
        "mixed": (
            f"Verdict: mixed -- the distance distribution at {v['rig_height_m']:.2f} m "
            f"rig height does not cleanly separate into a single surface or bottom "
            f"population (bimodality coefficient above {BIMODALITY_THRESHOLD:.3f}). "
            + (
                "Strength separates the two modes cleanly, so this may be filterable."
                if v["strength_separates"]
                else "Strength does not separate the two modes cleanly, so this may not be filterable."
                if v["strength_separates"] is not None else ""
            )
        ),
    }[v["verdict"]]
    print(f"\n  {sentence}")


# ----------------------------------------------------------------------
# Plots
# ----------------------------------------------------------------------

def make_plots(records, summaries, outdir):
    os.makedirs(outdir, exist_ok=True)
    finite = [s for s in summaries if not math.isnan(s["angle_deg"])]
    finite.sort(key=lambda s: s["angle_deg"])

    if finite:
        angles = [s["angle_deg"] for s in finite]

        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.scatter(angles, [100 * s["valid_fraction"] for s in finite])
        ax.set_xlabel("angle (deg)")
        ax.set_ylabel("valid fraction (%)")
        ax.set_title("Valid fraction vs angle")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "valid_fraction_vs_angle.png"), dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4.5))
        means = [s["mean_distance_cm"] for s in finite]
        stds = [0.0 if math.isnan(s["std_distance_cm"]) else s["std_distance_cm"] for s in finite]
        ax.errorbar(angles, means, yerr=stds, fmt="o", capsize=3)
        ax.set_xlabel("angle (deg)")
        ax.set_ylabel("mean distance (cm)")
        ax.set_title("Mean distance vs angle")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "mean_distance_vs_angle.png"), dpi=150)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.scatter(angles, [s["mean_strength"] for s in finite])
        ax.set_xlabel("angle (deg)")
        ax.set_ylabel("mean strength")
        ax.set_title("Strength vs angle")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "strength_vs_angle.png"), dpi=150)
        plt.close(fig)
    else:
        print("  (no bursts with a parseable angle, skipping the angle-based plots)")

    by_condition = defaultdict(list)
    for r in records:
        if r["strength"] is not None:
            label = r["condition_raw"].strip() or "(missing)"
            by_condition[label].append(r["strength"])

    if by_condition:
        fig, ax = plt.subplots(figsize=(8, 5))
        for label, vals in sorted(by_condition.items()):
            ax.hist(vals, bins=40, alpha=0.5, label=f"{label} (n={len(vals)})")
        ax.set_xlabel("strength")
        ax.set_ylabel("count")
        ax.set_title("Strength histogram per condition")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "strength_histogram_per_condition.png"), dpi=150)
        plt.close(fig)
    else:
        print("  (no strength samples at all, skipping the strength histogram)")


# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Phase 0 LiDAR-over-seawater bench test analysis")
    ap.add_argument("csv_path", help="CSV written by pc_csv_capture.py")
    ap.add_argument("--outdir", default=None,
                    help="where to save plots (default: analysis_out next to the csv)")
    args = ap.parse_args()

    if not os.path.exists(args.csv_path):
        sys.exit(f"No such file: {args.csv_path}")

    outdir = args.outdir or os.path.join(
        os.path.dirname(os.path.abspath(args.csv_path)), "analysis_out")

    records = load_records(args.csv_path)
    if not records:
        print("No data rows found in the CSV.")
        return

    n_malformed = sum(r["malformed_columns"] for r in records)
    if n_malformed:
        print(f"Note: {n_malformed} of {len(records)} rows had the wrong column count "
              f"and were padded/truncated rather than dropped.\n")

    bursts = group_bursts(records)
    summaries = [summarize_burst(bid, recs) for bid, recs in bursts.items()]
    summaries.sort(key=lambda s: (s["angle_deg"] if not math.isnan(s["angle_deg"]) else 1e9,
                                  str(s["burst_id"])))

    print_summary_table(summaries)
    print()
    print_boresight(fit_boresight(records))
    print()
    print_verdict(surface_vs_bottom_verdict(records))
    print()

    make_plots(records, summaries, outdir)
    print(f"Plots saved to {outdir}")


if __name__ == "__main__":
    main()
