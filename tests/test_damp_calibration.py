"""Independent synthetic responses exercise the public CSV command line."""
import csv
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import uuid
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/damp/fit_model.py"
FIELDS = "time_s,forward_mps,lateral_mps,yaw_radps,left_volts,right_volts"


def response(mode, direction=1):
    # Exact continuous exponential response to piecewise constant voltages.
    alpha, damping, friction = 1.7, 0.8, 0.24
    t, v = 0.0, direction * 0.15
    rows = []
    for i in range(350):
        voltage = direction * (1.0 + (i // 50) % 3)
        rows.append([t, v if mode == "drive" else 0, 0,
                     v if mode == "turn" else 0,
                     voltage if mode == "drive" else -voltage, voltage])
        dt = (0.005, 0.008, 0.013)[i % 3]
        terminal = (alpha * voltage - direction * friction) / damping
        v = terminal + (v - terminal) * math.exp(-damping * dt)
        t += dt
    return rows


def coast(initial=1.4, direction=1):
    # Exact solution has a constant-deceleration region and exponential tail.
    mu, knee, viscous = 0.65, 0.3, 0.2
    rows, t = [], 0.0
    for i in range(550):
        transition = max(0, math.log((initial + mu / viscous) / (knee + mu / viscous)) / viscous)
        v = (initial + mu / viscous) * math.exp(-viscous * t) - mu / viscous if t <= transition else min(initial, knee) * math.exp(-(mu / knee + viscous) * (t - transition))
        rows.append([t, 0, direction * v, 0, 0, 0])
        t += (0.007, 0.011, 0.014)[i % 3]
    return rows


class CalibrationTests(unittest.TestCase):
    def test_voltage_telemetry_mismatch(self):
        stream = io.StringIO()
        writer = csv.writer(stream)
        writer.writerow(FIELDS.split(",") + ["measured_left_volts", "measured_right_volts"])
        writer.writerows([row + [0, 0] for row in response("drive")])
        result = self.run_fit("drive", raw=stream.getvalue())
        self.assertNotEqual(result.returncode, 0)

    def run_fit(self, mode, rows=None, raw=None):
        path = SCRIPT.parents[2] / "tests" / ("calibration-" + uuid.uuid4().hex + ".csv")
        try:
            if raw is None:
                stream = io.StringIO()
                writer = csv.writer(stream)
                writer.writerow(FIELDS.split(","))
                writer.writerows(rows)
                raw = stream.getvalue()
            path.write_text(raw, encoding="utf-8")
            return subprocess.run([sys.executable, str(SCRIPT), str(path), "--mode", mode], capture_output=True, text=True)
        finally:
            path.unlink(missing_ok=True)

    def test_drive_and_turn_both_directions(self):
        for mode in ("drive", "turn"):
            for direction in (-1, 1):
                with self.subTest(mode=mode, direction=direction):
                    result = self.run_fit(mode, response(mode, direction))
                    self.assertEqual(result.returncode, 0, result.stderr)
                    data = json.loads(result.stdout)
                    for key, expected in (("alpha", 1.7), ("lambda", .8), ("sigma", .24)):
                        self.assertAlmostEqual(data["coefficients"][key], expected, delta=.002)
                    self.assertGreater(data["diagnostics"]["samples_used"], 300)
                    self.assertLess(data["diagnostics"]["rmse"], .001)

    def test_floor_both_directions(self):
        for direction in (-1, 1):
            result = self.run_fit("coast", coast(direction=direction))
            self.assertEqual(result.returncode, 0, result.stderr)
            data = json.loads(result.stdout)
            self.assertAlmostEqual(data["coefficients"]["mu"], .65, delta=.002)
            self.assertAlmostEqual(data["coefficients"]["knee_velocity"], .3, delta=.002)
            self.assertAlmostEqual(data["coefficients"]["viscous"], .2, delta=.002)

    def test_unidentifiable_data(self):
        cases = [("drive", [[i*.01, 1, 0, 0, 2, 2] for i in range(30)]),
                 ("drive", response("drive")[:45]),
                 ("coast", coast(initial=.1)), ("coast", coast()[:50])]
        for mode, rows in cases:
            result = self.run_fit(mode, rows)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "")
            self.assertIn("excitation", result.stderr)

    def test_small_measurement_noise(self):
        rows = response("drive")
        for i, row in enumerate(rows):
            row[1] += .00002 * math.sin(i * 1.7)
        result = self.run_fit("drive", rows)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertAlmostEqual(data["coefficients"]["alpha"], 1.7, delta=.01)
        self.assertAlmostEqual(data["coefficients"]["sigma"], .24, delta=.01)
        self.assertGreater(data["diagnostics"]["rmse"], .001)

    def test_rejects_nonphysical_voltage_sign(self):
        rows = response("drive")
        for row in rows:
            row[4], row[5] = -row[4], -row[5]
        result = self.run_fit("drive", rows)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, "")
        self.assertIn("nonpositive", result.stderr)

    def test_coast_rejects_power_and_coupled_motion(self):
        for column in (1, 3, 4, 5):
            rows = coast()
            rows[25][column] = .2
            result = self.run_fit("coast", rows)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("coast", result.stderr)

    def test_invalid_csv(self):
        good = response("drive")
        invalid = []
        for column, value in ((0, -1), (0, float("nan")), (1, float("inf")), (4, float("nan"))):
            rows = [row[:] for row in good]
            rows[10][column] = value
            invalid.append(rows)
        rows = [row[:] for row in good]
        rows[10][0] = rows[9][0]
        invalid.append(rows)
        invalid.append(good[:2])
        for rows in invalid:
            self.assertNotEqual(self.run_fit("drive", rows).returncode, 0)
        for raw in ("bad,header\n1,2\n", FIELDS + "\n0,no,0,0,0,0\n", FIELDS + "\n0,1,2\n"):
            self.assertNotEqual(self.run_fit("drive", raw=raw).returncode, 0)


if __name__ == "__main__":
    unittest.main()
