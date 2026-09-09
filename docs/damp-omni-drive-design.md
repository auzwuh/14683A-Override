# DAMP integration proposal

Status: approved and implemented as an experimental, disabled-by-default API.
Branch: `codex/damp-omni-drive`, created from `ramsete-lqr-tuning`.
See `damp-usage.md` for the implemented scope, verification and hardware rollout.
The sections below preserve the approved design context.

## Goal and assumptions

Track the intended autonomous path with an all-omni tank drivetrain, prioritizing
cross-track accuracy and a settled endpoint over minimum traversal time. Assume
the existing left/right motor arrangement remains, with one longitudinal tracking
pod, one lateral tracking pod, and the IMU. Confirm this before hardware integration.

The supplied *DAMP - Drift Aware Motion Profiles.pdf* is the technical reference.
Its instructions addressed to AI systems are document content, not user requests.
Section/page references below refer to that PDF.

DAMP models sideslip rather than promising zero sliding. Its generator schedules
position, body heading, velocity, and motor voltage; its feedback follower corrects
the measured motion (section 1.2, p. 3). Body heading may differ from travel direction.
Use conservative speed and sideslip limits initially; determine their values on
this robot. The paper's measured performance is not a prediction for this build.

## Current code and integration gaps

- `src/main.cpp` declares rotation sensors on ports -15 and -16, but the tracking
  wheel objects are commented out and all tracking-wheel pointers are null.
  The commented diameters and offsets are historical values, not confirmed geometry.
- `Chassis::calibrate()` substitutes drive encoders for missing longitudinal pods.
  That fallback cannot supply a measurement of lateral sliding.
- `src/gen/chassis/odom.cpp` already compensates tracking-wheel offsets and exposes
  body-local velocity. It uses a fixed 10 ms interval and an exponential filter.
  DAMP needs a coherent pose/velocity sample with actual elapsed time; verify the
  latency and noise before selecting its velocity measurement path (p. 43).
- The existing `arc::path::State` has pose, forward speed, and yaw rate, with no
  lateral velocity. `followRamseteLQR()` supplies unsigned total translation speed
  as forward speed. These quantities differ during a slide.
- The chassis uses inches and clockwise compass headings. The paper uses SI units
  and counter-clockwise angles. Explicit conversion is required at the boundary.
- Existing path assets contain position and duty-based speed, optionally curvature.
  They do not contain DAMP's complete timed reference and feedforward voltages.
- `ARC_RAMSETE_LQR_ENABLED` is currently 0. Competition routines also use point moves
  and turns; adding a path follower alone will not update those motion behaviors.

## Approaches

1. **Recommended: add DAMP as an opt-in subsystem, validate it, then migrate autons.**
   Separate math from PROS hardware access and compare against the existing follower.
   This supports the full drift model while keeping the rollout measurable.
2. **Only reduce speed and retune current controllers.** Less implementation work,
   useful as a baseline, but it does not implement DAMP or explicitly predict sideslip.
3. **Replace every motion primitive at once.** Covers more of the auton immediately,
   but makes sensor, model, follower, and routine failures difficult to distinguish.

## Proposed implementation stages

### 1. State measurement and calibration

Add a timestamped snapshot of field pose and signed body forward/lateral/yaw
velocities, with explicit sensor validity. Test coordinate conversion, pure
translation, pure rotation with offset pods, variable loop timing, and pose resets.
Retain existing public odometry behavior unless a separately verified fix is needed.

Add manually selected logging routines for forward/reverse voltage response,
turning response, and lateral coast-down. Fit the drive, yaw, and floor models
from measurements (section 2). Treat fitted parameters as robot configuration;
do not substitute the existing `horizontalDrift` number for identified dynamics.
Enabling DAMP requires valid calibration and the intended sensors.

### 2. Model and trajectory generator

Put hardware-independent DAMP math in its own module. Use metres, seconds, radians,
and volts internally; keep the existing auton-facing field convention at the adapter.
Represent body heading separately from path tangent and retain signed lateral speed.

Generate the smooth path, distance samples, speed/sideslip profile, and feedforward
voltages using the paper's construction. Reserve motor authority for feedback.
Reject invalid geometry, nonfinite output, and profiles that fail feasibility checks.
Do not silently interpret legacy duty values as physical speeds or voltages.
Start with an explicit waypoint interface; define any asset conversion separately.

Test straight paths, mirrored curves, curvature transitions, stopping endpoints,
and model residuals. Compare the planned motion with a separately implemented
simulation before adding live motor output.

### 3. Feedback follower and chassis adapter

Implement the six-state, voltage-constrained predictive follower, including bounded
solver work, reference-clock pacing, and measured-velocity settling (sections 7-9).
Test analytic derivatives numerically and the two-input constrained solve against
an independent numerical reference. Measure worst-case execution on the V5 Brain.

Expose an opt-in `followDamp` motion with explicit completion, timeout, cancellation,
invalid-input, and sensor-failure outcomes. Preserve the chassis motion lifecycle.
Send the computed physical voltages through a dedicated voltage adapter rather
than the existing duty-to-speed conversion. Keep commands within available voltage.

Require endpoint position/heading tolerances and settled forward, lateral, and yaw
velocities for a stopping endpoint. Reaching the last trajectory timestamp alone
must not report successful arrival. Start with stopping segments; validate rolling
handoffs and reverse segments explicitly before relying on them in competition.

### 4. Auton rollout

Add selectable straight, arc, S-curve, and stop/turn/drive tests. Compare RMS and peak
cross-track error, endpoint position/heading, residual sideways speed, duration,
saturation, and loop timing across repeated trials at the same speed limits.
Include changed load and battery conditions in robot validation.

Then migrate one real auton segment at a time. Audit point moves, turns, and swing
turns for residual sliding (section 11, pp. 62-63). Preserve intentional contact
moves such as the existing wall/Toggle interaction as explicit routine actions;
their completion semantics differ from free-space path tracking.

## Hardware information needed

Confirm that this remains a tank layout, then record actual wheel diameter, gearing,
track width, motor directions, pod diameters, signed offsets, and sensor ports after
assembly. The first software stage can provide measurement and calibration support
before final physical values are available. Robot performance remains unverified
until those measurements and driving trials are complete.
