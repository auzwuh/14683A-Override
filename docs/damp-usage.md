# DAMP on the omni drivetrain

The experimental implementation is on `codex/damp-omni-drive`. It combines a
drift-aware path generator with feedback from signed forward/lateral pod velocity
and IMU yaw rate. It reduces tracking error by changing both wheel voltage and
reference timing. All-omni tank drive still cannot command sideways force directly;
neither the model nor the controller guarantees zero drift.

`ARC_DAMP_ENABLED` in `include/gen/features.hpp` defaults to **0**. Both chassis
entry points return `disabled` in this configuration. Existing competition auton
routines and the commented pod hardware configuration have not been converted.
The odometry implementation now publishes coherent snapshots under a mutex;
when both unpowered pods and IMU are present, pose and velocity share IMU heading.

## Robot setup and calibration

Configure the actual longitudinal and lateral unpowered tracking wheels plus IMU
in `src/main.cpp`, measure their offsets and directions, and calibrate odometry.
The historical commented ports and dimensions must not be treated as measured
values. Check pure forward, lateral and rotation motion before collecting data.
`getOdomSnapshot()` must report valid samples before either DAMP entry point runs.

After enabling the feature, explicitly call `chassis.logDampCalibration(steps,
output, voltageLimit)` from a manually selected test routine. `output` is an open
CSV stream (or `stdout`); each `CalibrationStep` contains a duration in milliseconds
and signed left/right **volts**. The caller supplies the experiment and voltage
limit; there is no automatically selected motor test. The method serializes with
other chassis motions, stops both sides when it finishes or faults, and returns a
`MotionResult`. Keep unrelated actuator code from writing drivetrain output during
the test. Zero-voltage steps provide a sideways coast recording after push release.

The logger records requested voltages held until the next row plus mean measured
motor voltages in extra columns. Verify telemetry polarity and timing on the robot.
The fitter rejects discrepancies over 0.25 V; that is a coarse rejection threshold,
not a measurement accuracy guarantee. Reject runs affected by voltage limiting,
sensor latency, pushes within the fitted interval, or logging gaps. Do not simply
remove telemetry columns to bypass rejection. Serial/SD throughput and snapshot
latency must be measured before relying on the fits. See `damp-calibration.md` for
the drive, yaw and floor experiments and fit diagnostics.

Copy reviewed fitted coefficients into `damp::Model`: drive and turn use
`{alpha, lambda, sigma, epsilon}`; floor uses `{mu, knee, viscous}`. Epsilon is a
numerical smoothing velocity requiring validation. Defaults are deliberately
invalid. The Euler follower rejects models whose fastest friction relaxation
rate times the prediction step is at least one; decrease the prediction step or
review smoothing if rejected.

## Following a path

Include `gen/damp/motion.hpp`. Generate outside the time-critical motion loop:

```cpp
// model, limits and followerLimits contain measured/validated robot settings.
auto generated = arc::damp::generate(
    {arc::damp::waypointInches(0, 0),
     arc::damp::waypointInches(24, 6),
     arc::damp::waypointInches(48, 24)}, model, limits);
if (!generated.ok()) return; // inspect generation status and residual diagnostics
arc::damp::MotionParams motion;
motion.follower = followerLimits;
motion.timeoutMs = 15000;
auto result = chassis.followDamp(generated.trajectory, model, motion);
if (result.status != arc::damp::MotionStatus::succeeded) return;
```

The example is an API sketch, not calibrated robot configuration. `GeneratorConfig`
requires speed, yaw, voltage, launch/brake acceleration, spacing and endpoint taper
settings. Speeds/distances are metres and seconds internally; headings are CCW
radians from field +X. `waypointInches` preserves existing field coordinates.
Set the actual starting robot pose using the existing inches/compass API. Reverse
generation uses `limits.reverse=true`; the robot starts facing opposite the path
tangent. Waypoint headings are determined by geometry, with stopping endpoints.

Reserve correction headroom through generator `correctionMargin` and set follower
`maxVoltage` to the physical limit. The adapter refuses a battery below that limit
instead of silently applying a different saturation model. It also stops on invalid
or stale snapshots, motor API errors, cancellation, timeout or invalid optimizer
results. Completion requires position, heading and all three velocities to remain
within their tolerances for the settling duration. A timeout is not success.
The call is synchronous and participates in the existing chassis motion queue.

## Scope relative to the supplied paper

- Six-state signed dynamics, three-parameter lateral friction, C4 quintic waypoint
  geometry, drift-aware speed/sideslip passes, voltage feedforward, constrained
  iLQR and paced reference time are implemented.
- The spline continuity solve is dense with a 64-waypoint cap, rather than the
  paper's linear-time banded solve. Arc lookup uses binary search and bounded
  inversion. Generate before autonomous execution; profile memory and time.
- The follower predicts field pose with analytic Euler derivatives and costs
  position error in reference body axes. This differs from the paper's rotating
  error-state formulation. Horizon/iterations have fixed upper bounds and storage
  is allocated before the robot loop. Real V5 execution time is unmeasured.
- Generation checks sustained planar acceleration residual, rather than only the
  lateral residual. This checks both forward and lateral dynamics but is not a
  proof of path feasibility. Only stopping trajectories are supported; there is
  no obstacle/collision checking or automatic migration of contact-based autons.

## Verification and remaining hardware work

Run `tests/run_damp_tests.ps1` with a Python executable via `-Python` if needed.
Six C++ test programs cover dynamics, spline continuity, signed measurement,
trajectory symmetry, Jacobians/box constraints, settling and disturbed tracking.
Eight Python test groups cover identification and rejected input. Forward/reverse
curved tracking on a synthetic RK4 plant with a 0.08 m/s lateral disturbance had
peak reference-position error below 8 mm. This is a matched-model host test, not
a claim about robot performance or general disturbance recovery.

The existing path-following host regression suite passes. All new DAMP and modified
odometry modules compile with the installed PROS ARM toolchain, including the
enabled motor adapter. The full firmware build is blocked by missing existing
lift/arm constants in `src/autons.cpp` (including `LIFT_TOL` and `ROT_TOL`).

Before competition use: resolve that build configuration, identify the assembled
robot, measure loop timing, test slow straight/reverse/curve paths, inject small
disturbances, and validate endpoint settling across battery levels. Convert each
auton segment only after that validation; retain explicit contact motions where
the routine intentionally touches the field.
