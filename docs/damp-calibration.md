# DAMP calibration

The fitter uses Python's standard library and emits measured coefficients, never
default robot parameters. Run each experiment separately:

```powershell
python scripts/damp/fit_model.py drive.csv --mode drive > drive-model.json
python scripts/damp/fit_model.py turn.csv --mode turn > turn-model.json
python scripts/damp/fit_model.py coast.csv --mode coast > floor-model.json
python tests/test_damp_calibration.py
```

The positional argument is an existing CSV file. `--mode` is required and is one
of `drive`, `turn`, or `coast`. Success writes one JSON object to stdout and exits
0. Failure writes a diagnostic to stderr, leaves stdout empty, and exits 2.
Shell redirection can leave an empty output file after failure; check the exit
status before importing coefficients. JSON is a calibration report, not an
automatically loaded robot configuration.

## Measurement contract

Required unique columns (extra named columns are allowed):

```csv
time_s,forward_mps,lateral_mps,yaw_radps,left_volts,right_volts
```

Time is elapsed seconds, strictly increasing and nonnegative. All required values
must be finite numbers. Body forward and leftward velocity are signed metres per
second. Yaw is counterclockwise radians per second. Left/right voltage uses the
robot convention in which positive voltage on both sides drives forward. Each
row records the voltage held from that timestamp until the next row. Log actual
applied voltage after saturation; do not substitute duty, millivolts, requested
voltage before clipping, compass clockwise yaw, or unsigned total speed.

Use coherent odometry snapshots with both unpowered tracking pods and the IMU.
The opt-in `Chassis::logDampCalibration` entry point produces this CSV plus measured
motor-voltage columns. See `damp-usage.md` for execution and telemetry caveats.
If telemetry columns exist, the fitter rejects command discrepancies above 0.25 V.
First verify actual pod geometry, ports, directions, offset compensation, and
measurement latency. The repository's commented geometry is not calibration.
Drive-encoder fallback cannot identify lateral drift. Exclude invalid snapshots,
pose resets, collisions, pushes during the fitted interval, and gaps in logging;
start a separate CSV after each discontinuity. Do not concatenate runs with a
velocity jump and treat the jump as one time interval.

## Manually selected experiments

Run only after physical setup and a safe test area have been checked. These are
operator-selected procedures; running the Python tests does not move the robot.

1. **Drive:** command equal left/right voltage at several magnitudes and retain
   both acceleration transients and steady motion. Keep lateral velocity and yaw
   negligible. Collect forward and reverse trials to check symmetry. A single
   constant voltage cannot distinguish its gain from Coulomb friction, even if
   speed changes.
2. **Turn:** apply opposite voltages about the chassis centre at several
   magnitudes, preserving their signs. Record transient and steady yaw in both
   directions with negligible translation.
3. **Coast:** set zero motor voltage, release a sideways push, and log free decay
   after contact ends. Keep forward velocity and yaw negligible. Start above the
   friction knee and retain a clean low-speed tail. The region above the knee
   identifies the constant friction term and viscous slope; the region below
   identifies the knee. A purely exponential tail or only high-speed data cannot
   identify all three coefficients.

`--isolation-tolerance` defaults to `0.001`. Coast rejects any row whose absolute
forward m/s, yaw rad/s, left volts, or right volts exceeds this numerical
tolerance. This shared threshold has different units for each channel. Select
it from measured sensor noise, and never increase it to admit appreciable coupled
motion. The coast fitter includes no forward/yaw coupling compensation. Drive
and turn isolation must be checked by the operator.

## Models and output

Drive uses `Vd=(left_volts+right_volts)/2`; turn uses
`Vt=(right_volts-left_volts)/2`. Both fit:

```text
dv/dt = alpha * V - lambda * v - sigma * sign(v)
```

`coefficients` contains positive `alpha`, `lambda`, and `sigma`. For drive their
units are m/s²/V, 1/s, and m/s². For turn replace metres with radians.
The numerical smoothing epsilon used by the motion model is not a fitted physical
coefficient; this fitter uses signed moving intervals and omits zero crossings.

Coast fits:

```text
dv_left/dt = -mu * clamp(v_left/knee_velocity, -1, 1) - viscous * v_left
```

Its coefficients are `mu` (positive m/s²), `knee_velocity` (positive m/s), and
`viscous` (nonnegative 1/s). Map `knee_velocity` to the C++ floor model's `knee`.
Do not substitute the chassis `horizontalDrift` setting for these values.

Each report includes `mode` and `diagnostics`: `samples_read`, `samples_used`
(interval count), `intervals_skipped`, acceleration `rmse`, `max_abs_residual`,
and `relative_rmse`. Coast also reports counts below 0.8 times and above 1.2 times
the fitted knee. These counts deliberately omit the transition neighbourhood.

The implementation estimates acceleration by adjacent differences using actual
intervals, evaluates speed at the interval midpoint, and applies column-scaled
least squares. Coast searches the knee and fits its two linear coefficients,
including the zero-viscosity boundary. This is a numerical regression approach,
not the paper's steady-response plus exponential-fitting procedure. Differencing
amplifies velocity noise, and midpoint discretization introduces sampling error;
collect clean, sufficiently frequent measurements and compare separate trials.
No automatic filtering is applied, and all intervals receive equal weight.

Fits reject fewer than 12 rows or 10 usable intervals, nearly dependent
regressors, nonphysical drive/turn coefficients, missing floor regions, a
negligible floor friction term, and residual RMS greater than 25% of measured
acceleration RMS. These are numerical screening rules, not a confidence interval
or proof of a robot model. Near-zero velocity intervals are omitted below
1e-6 in the corresponding SI velocity unit. Review residuals and repeated fits,
especially with noisy sensors, different battery conditions, and changed load,
before manually transferring coefficients into the disabled DAMP configuration.
