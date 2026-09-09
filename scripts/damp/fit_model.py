#!/usr/bin/env python3
"""Fit isolated DAMP responses using only the Python standard library."""
import argparse
import csv
import json
import math
import sys

FIELDS = "time_s forward_mps lateral_mps yaw_radps left_volts right_volts".split()


def read_samples(path):
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames) or not set(FIELDS).issubset(reader.fieldnames):
            raise ValueError("CSV must contain unique required columns: " + ",".join(FIELDS))
        rows = []
        for line, raw in enumerate(reader, 2):
            if None in raw or any(raw.get(k) is None for k in reader.fieldnames):
                raise ValueError(f"malformed CSV row {line}")
            try:
                row = [float(raw[k]) for k in FIELDS]
            except (ValueError, TypeError) as error:
                raise ValueError(f"nonnumeric CSV row {line}") from error
            if not all(math.isfinite(x) for x in row):
                raise ValueError(f"nonfinite CSV row {line}")
            for name, command in (("measured_left_volts", row[4]), ("measured_right_volts", row[5])):
                if name in raw:
                    measured = float(raw[name])
                    if not math.isfinite(measured) or abs(measured-command) > 0.25:
                        raise ValueError(f"voltage telemetry differs from command at row {line}; check signs, latency and current limiting before fitting")
            if row[0] < 0 or (rows and row[0] <= rows[-1][0]):
                raise ValueError(f"timestamps must be nonnegative and strictly increasing (row {line})")
            rows.append(row)
    if len(rows) < 12:
        raise ValueError("insufficient excitation: at least 12 samples required")
    return rows


def least_squares(x, y):
    """Column-scaled modified Gram-Schmidt; reject nearly dependent regressors."""
    count = len(x[0])
    columns = [[row[j] for row in x] for j in range(count)]
    scales = [math.sqrt(sum(v*v for v in col)) for col in columns]
    if min(scales) <= 1e-12:
        raise ValueError("insufficient excitation: zero regressor")
    q, r = [], [[0.0]*count for _ in range(count)]
    for j, column in enumerate(columns):
        v = [a/scales[j] for a in column]
        for k in range(j):
            r[k][j] = sum(a*b for a, b in zip(q[k], v))
            v = [a-r[k][j]*b for a, b in zip(v, q[k])]
        r[j][j] = math.sqrt(sum(a*a for a in v))
        if r[j][j] < 1e-3:
            raise ValueError("insufficient excitation: dependent regressors")
        q.append([a/r[j][j] for a in v])
    c = [sum(a*b for a, b in zip(col, y)) for col in q]
    for j in range(count-1, -1, -1):
        c[j] = (c[j] - sum(r[j][k]*c[k] for k in range(j+1, count))) / r[j][j]
    return [a/b for a, b in zip(c, scales)]


def fit(rows, mode, isolation_tolerance):
    if mode == "coast" and any(abs(row[j]) > isolation_tolerance for row in rows for j in (1, 3, 4, 5)):
        raise ValueError("coast requires zero voltage, forward velocity and yaw within --isolation-tolerance")
    index = {"drive": 1, "turn": 3, "coast": 2}[mode]
    speeds, targets, voltages = [], [], []
    skipped = 0
    for a, b in zip(rows, rows[1:]):
        # A sample's voltage is held until the following timestamp.
        if a[index]*b[index] <= 0 or abs(a[index]+b[index]) < 2e-6:
            skipped += 1
            continue
        speeds.append((a[index]+b[index])/2)
        targets.append((b[index]-a[index])/(b[0]-a[0]))
        voltages.append((a[4]+a[5])/2 if mode == "drive" else (a[5]-a[4])/2)
    if len(speeds) < 10:
        raise ValueError("insufficient excitation: fewer than 10 moving intervals")
    extra = {}
    if mode != "coast":
        x = [[u, -v, -math.copysign(1, v)] for u, v in zip(voltages, speeds)]
        values = least_squares(x, targets)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("fit has nonpositive drive/yaw coefficients; check excitation, signs and model")
        coefficients = dict(zip(("alpha", "lambda", "sigma"), values))
        predictions = [sum(a*b for a, b in zip(row, values)) for row in x]
    else:
        magnitude = [abs(v) for v in speeds]
        decay = [-a*math.copysign(1, v) for a, v in zip(targets, speeds)]

        def candidate(log_knee):
            knee = math.exp(log_knee)
            x = [[min(v/knee, 1), v] for v in magnitude]
            try:
                mu, viscous = least_squares(x, decay)
            except ValueError:
                return float("inf"), 0, 0
            if viscous < 0:
                viscous = 0
                mu = sum(row[0]*y for row, y in zip(x, decay))/sum(row[0]**2 for row in x)
            if mu <= 0:
                return float("inf"), mu, viscous
            error = sum((mu*row[0]+viscous*row[1]-y)**2 for row, y in zip(x, decay))
            return error, mu, viscous

        low, high = math.log(min(magnitude)), math.log(max(magnitude))
        grid = [low+(high-low)*i/160 for i in range(161)]
        best = min(range(len(grid)), key=lambda i: candidate(grid[i])[0])
        left, right = grid[max(0, best-1)], grid[min(160, best+1)]
        for _ in range(65):
            p, q = left+(right-left)/3, right-(right-left)/3
            if candidate(p)[0] < candidate(q)[0]:
                right = q
            else:
                left = p
        log_knee = (left+right)/2
        error, mu, viscous = candidate(log_knee)
        knee = math.exp(log_knee)
        below = sum(v < knee*.8 for v in magnitude)
        above = sum(v > knee*1.2 for v in magnitude)
        # Both regions and their different slopes must be visible. A purely
        # exponential trace identifies only (mu/knee + viscous).
        if not math.isfinite(error) or below < 5 or above < 5 or mu < max(decay)*1e-3:
            raise ValueError("insufficient excitation: floor needs distinct knee and plateau regions")
        # Local parameter Jacobian guards three-parameter identifiability.
        least_squares([[min(v/knee, 1), v, -mu*v/knee**2 if v < knee else 0] for v in magnitude], decay)
        coefficients = {"mu": mu, "knee_velocity": knee, "viscous": viscous}
        predictions = [-math.copysign(mu*min(abs(v)/knee, 1)+viscous*abs(v), v) for v in speeds]
        extra = {"samples_below_knee": below, "samples_above_knee": above}
    residuals = [a-b for a, b in zip(targets, predictions)]
    rmse = math.sqrt(sum(r*r for r in residuals)/len(residuals))
    scale = math.sqrt(sum(a*a for a in targets)/len(targets))
    if not math.isfinite(rmse) or scale <= 1e-10 or rmse > .25*scale:
        raise ValueError("inadequate model fit: residual exceeds 25% of acceleration RMS")
    return {"mode": mode, "coefficients": coefficients,
            "diagnostics": {"samples_read": len(rows), "samples_used": len(speeds),
                            "intervals_skipped": skipped, "rmse": rmse,
                            "max_abs_residual": max(abs(r) for r in residuals),
                            "relative_rmse": rmse/scale, **extra}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="CSV with SI velocities, seconds and volts")
    parser.add_argument("--mode", required=True, choices=("drive", "turn", "coast"))
    parser.add_argument("--isolation-tolerance", type=float, default=1e-3,
                        help="coast maximum absolute forward m/s, yaw rad/s and volts (default .001)")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.isolation_tolerance) or args.isolation_tolerance < 0:
            raise ValueError("isolation tolerance must be finite and nonnegative")
        result = fit(read_samples(args.csv), args.mode, args.isolation_tolerance)
        output = json.dumps(result, indent=2, allow_nan=False)
    except (OSError, ValueError, OverflowError, csv.Error) as error:
        print(f"calibration error: {error}", file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
