// sim_config.hpp - every number the simulator invents, in one place.
//
// The robot code (src/, include/) is compiled unchanged. Values it leaves as NAN
// or placeholders are assigned from sim_main.cpp (applyRobotConstants) BEFORE
// initialize() runs. Everything else here is the sim's own model of the hardware
// and the field (TRUTH). The robot only sees truth through the fake sensors.
//
// ---------------------------------------------------------------------------
// Robot constants the sim assigns (robot code leaves them NAN / unset)
// ---------------------------------------------------------------------------
// | constant                   | sim value       | why                                                    |
// |----------------------------|-----------------|--------------------------------------------------------|
// | dr4b::kG                   | measured in sim | volts that hold a Cup+Pin at 35 deg on THIS seed's     |
// |                            | (about 3.4 V)   | plant (world.cpp liftHoldVolts), times a tuning error  |
// |                            |                 | of +-15 % per seed. The plant's gravity curve is NOT   |
// |                            |                 | the code's gravityShare() curve (mid-stage mass).      |
// | dr4b::kP                   | 0.8 V/deg       | settles inside 1.5 deg, under 0.5 deg overshoot        |
// | dr4b::kD                   | 0.012 V/(deg/s) | trims overshoot; back-EMF supplies most damping        |
// | dr4b::kTolDeg              | 1.5 deg         | 0.67 in at the peak slope vs 0.4 in drop clearance     |
// | dr4b::kCarry               | 4 deg (header)  | object bottom 1.5 in up, lift's back corners 11.5 in out |
// | dr4b::kToggleUp            | about 40 deg    | TRUE rise 15.68 in: the claw underside (0.25 in below  |
// |                            |                 | the held Pin) is 1.0 in over the top of the turning    |
// |                            |                 | Toggle, 14.43 in. Solved on each seed's own lift, the  |
// |                            |                 | way a team raises it until it clears                   |
// | toggle::kFlipTurns         | NOT filled      | 2.0 in the code. --flip-turns N sets it from the sim   |
// |                            |                 | side for the "which value ends red" sweep only         |
// | dr4b::kGoalTopIn[Tall]     | 8.77 in         | ATTICUS draw.py Tall Goal height                       |
// | dr4b::kSeatIn[Tall]        | 5.605 in        | 8.77 - 3.165 nest                                      |
// | rclaw::kSeesObjectMm       | 70 mm           | between seated (40) and an empty throat (over 100)     |
// | localizer.mounts[i].biasIn | calibrated      | printReadings procedure, parked 6.0 in from a wall:    |
// |                            |                 | true distance minus median reading (0 with --no-noise) |
// | localizer.kFrontBumperIn   | 6.50 in         | robot origin (traction axle) to the front rail ends    |
// | localizer.kBackBumperIn    | 8.55 in         | robot origin to the back of the frame. A low lift      |
// |                            |                 | touches first: backBumperIn(), 10.27 in folded         |
// | WallLocalizer::kCloseMv .. | NOT filled      | 3000 / 1800 mV and 0.10 in lead, as the code has them  |
// Left as the code has them: kGoalTopIn/kSeatIn Short+Alliance (3.25 / 0.085), kHoldMv 0,
// kReleaseMs 300, kLoader 0, kFloor 0, kUnitBelowPinIn 0, kClawBelowObjIn 0.25,
// kDropClearIn 0.4, kPitchIn 3.32, kLiftTable empty, clock limits 54000 / 53500, all ports.
// Cannot be set from outside (constructor arguments): IMU scales 1.0, tracking wheel
// 2.0 in / +0.364 in. Their truth is varied with --imu-gain and --wheel-dia instead.
//
// FRAMES. The robot code's origin is the middle of the TRACTION wheel axle. The sim's TRUE pose
// (body.x, body.y, what every log and error compares against odometry) is that same physical
// point. The CAD measures from the FRAME centre, kAxleFwdIn (1.5 in) behind it, and every robot
// dimension below is kept in that CAD frame ("ahead of the frame centre"); world.cpp shifts by
// kAxleFwdIn when it places a part in the world. Nothing here is taken from what the robot code
// believes (its 6.50 / 8.55 / 6.25 / -1.0 / 9.5 are the same parts measured from the axle).
//
// FINAL CAD (dr4b (2).step, CAD_FACTS.md). Changed from it: lift lengths and rest directions,
// rear contact 7.05 (and the raised lift's overhang), wall top 11.58, tracking wheel offset
// +0.364, lift top stop 105, the body origin and turning centre on the traction axle
// (kAxleFwdIn, kPivotFwdIn), and the whole Toggle spinner / Toggle prism block. Checked against
// it and left: front rail 8.0, half width 6.8, track 10.63 (CAD 10.616), wheels 2.75 and 2.0 in,
// 36:48 drive, 4:1 lift crank (the CAD shows 12T:48T only), front omni axle 5.0, sensor mounts
// (planned spots 7.75 / 0.5 ahead of the frame centre, no sensor in the CAD; the new brace and
// the chain that cross the side sensor spots are NOT modelled). Not in the CAD at all, still
// ASSUMED: claw reach 11.0 (= 9.5 ahead of the axle, the route's S_LOAD) and every claw number,
// masses, CG (at the frame centre), friction.
//
// Sweep-only flags that set ROBOT constants from the sim side (never part of an "as written" run):
// --flip-turns N (toggle::kFlipTurns), --close-lead IN / --close-mv MV / --close-slow-mv MV
// (WallLocalizer::kCloseLeadIn / kCloseMv / kCloseSlowMv). Ablation: --no-claw-wall lets the claw pass
// through walls and the Toggle again, as the pre-CAD sim did.
//
// --tuned-drive replaces ONLY the two drive PID gain sets (chassis.lateralPID and
// chassis.angularPID, public members) with kTunedLateral / kTunedAngular in sim_main.cpp.
#pragma once

namespace simcfg {
constexpr double IN = 0.0254;           // m per inch
constexpr double G = 9.81;

// ---- field (arc frame: origin centre, +y red wall, heading 0 = +y, clockwise +)
constexpr double kHalf = 140.41 / 2.0;  // wall faces at +-70.205
constexpr double kScale = 140.41 / 144.0;
constexpr double kWallHeightIn = 11.58; // final CAD: top of the wall's top rail (was 11.5, assumed)

// ---- chassis (truth)
constexpr double kMassKg = 5.67;        // 12.5 lb ASSUMED (ATTICUS config says 11.5). +-8 % per seed
constexpr double kChassisYawJ = 0.10;   // kg m^2 ASSUMED, plus carriage mass x reach^2 (0.18 lift down). +-25 % per seed
constexpr double kTrackIn = 10.63;
constexpr double kWheelDiaIn = 2.75;
constexpr double kDriveRatio = 48.0 / 36.0;   // motor:wheel, 600 -> 450 rpm
constexpr double kGearEff = 0.90;       // ASSUMED. Motor driving: torque x eff. Mechanism driving the motor: torque / eff
constexpr double kDriveFricN = 1.9;     // ASSUMED constant drag per side at the ground (axles, gear mesh), independent of
                                        // load. Puts breakaway near move(10) of 127 (selftest row a3). +-50 % per seed,
                                        // --drive-fric N. MEASURE: smallest move() that starts the robot, and coast distance.
constexpr double kMu = 0.90;            // rolling-direction friction of all 6 wheels (traction limit)
constexpr double kTractionShare = 1.0 / 3.0;  // weight on the two centre traction wheels ASSUMED
constexpr double kOmniLatMu = 0.05;     // omni rollers sideways
constexpr double kRollResist = 0.03;
constexpr double kScrubLeverIn = 0.25;  // yaw friction = mu*m*g*lever. ASSUMED and NOT validated: turn overshoot and the
                                        // --tuned-drive gains hinge on it (90 deg turn overshoots 24 deg at 0.25, 0 at 3.0)
                                        // and gate a2 cannot tell these plants apart. --no-noise uses this value, noise
                                        // seeds draw log-uniform from kScrubLo..kScrubHi, --scrub IN fixes it.
                                        // MEASURE: spin at move(127/-127), cut power, degrees to stop (selftest row a2c).
constexpr double kScrubLo = 0.25, kScrubHi = 2.0;
constexpr double kFrontIn = 8.0;        // frame centre to front rail end (final CAD 7.999) = 6.50 ahead of the traction axle
constexpr double kRearIn = 7.05;        // frame centre to rearmost part, lift DOWN: the 1x1 angles on the back of the frame
                                        // (final CAD 7.047; was 8.4, the old lift overhang) = 8.55 behind the axle. --rear IN
constexpr double kRouteRearIn = 7.05;   // the FRAME as the robot code has it (kBackBumperIn 8.55 from the axle). The low lift is in dr4b::rearIn()
// The raised lift does overhang the back: the middle link swings out behind the frame (final CAD: tower pivot 1.987 in
// ahead of the track centre, link edge 1.35 in behind the middle pivot: 6.43 behind in the CAD pose). Used for walls and
// the Midfield only; it passes over Goals and Cups. 7.37 at kCarry (8 deg), 10.3 with stage 1 level (51 deg).
constexpr double kTowerPivotFwdIn = 1.987, kTowerPivotUpIn = 11.875, kMidLinkBehindIn = 1.35, kLiftHalfWIn = 4.25;
// 19 Nov CAD: the rearmost part is the rear end of the upper stage 1 bar, 12.5 in from the tower pivot, bar 1 in high.
// 8.77 behind the frame centre folded (10.27 from the axle), 10.5 with stage 1 level, over the wall top from there.
constexpr double kBarEndIn = 12.5, kBarHalfIn = 0.5;
// Body origin and turning centre. Final CAD (same in the old one): the two traction wheels are 1.499 in AHEAD of the
// frame centre (omnis at +-5.0). The TRUE pose is the middle of that axle (kAxleFwdIn, geometry). The traction wheels
// are the only wheels that hold the robot sideways, so sideways friction acts at kPivotFwdIn ahead of the frame centre:
// the same point, so the robot turns about its true origin. Before the final CAD the sim's origin was the frame centre
// and it turned about that. --pivot-fwd IN moves only the friction point (0 = a robot that really turns about its
// frame centre while the code and the route assume the axle).
// The CG is ASSUMED on the traction axle (kCgFwdIn), so a spin needs no net tyre force and the robot turns exactly about
// the axle at any rate; with that the drive dynamics are the pre-CAD sim's, only the geometry around the pivot moved.
// Nobody has weighed the robot. The sim's own masses (kCarriageKg at the claw, the rest centred on the frame) put the CG
// 2.0 in ahead of the frame centre with the lift down and 0.7 in with it up, both inside 0.8 in of the axle. A CG off
// the axle makes the turning centre walk sideways in proportion to the yaw rate (selftest p1 with --cg-fwd 0: 0.6 in
// at 140 deg/s) and a fast spin slips. --cg-fwd IN
constexpr double kAxleFwdIn = 1.5;
constexpr double kPivotFwdIn = kAxleFwdIn;
constexpr double kCgFwdIn = kAxleFwdIn;
constexpr double kChassisCgHIn = 3.5, kFrontAxleIn = 5.0, kCarriageCgUpIn = 4.0;   // ASSUMED, only for the tip-risk log
constexpr double kHalfWidthIn = 6.8;    // 13.6 in wide frame
// Final CAD: the front is open between the drive rails (inner faces +-4.20). A Goal base (r 3.22) slides in
// until it seats in the V notch of the front plate with its centre 4.72 in ahead of the traction axle.
// ponytail: the V is modelled as a flat edge that stops the base at the same place on the centre line, it
// does not centre a base that comes in off line. Model the two V edges if off-line seating matters.
constexpr double kNotchHalfWIn = 4.20;
// 25 Nov: the V plate is replaced by two round aligner pieces that seat the base 6.70 in ahead of the axle (was 4.72).
// Still a flat edge here. The real pieces are a 5.11 in radius arc: by the field drawing the Loader chute corners clear
// them by 0.45 in at the grab stop.
constexpr double kNotchFlatIn = 6.70 + 1.5 - 3.22;   // frame centre to that flat edge (kAxleFwdIn = 1.5, kGoalR = 3.22)
constexpr double kContactK = 50000.0;   // N/m penalty contact
constexpr double kContactC = 500.0;     // N s/m, approach only

// ---- V5 11 W smart motor at the cartridge output
// tau = clamp(2*stall*(V/12 - w/wFree), +-stall) - viscous. The +-stall clamp is the 2.5 A
// current limit: it binds below half speed and peak power is 11 W. Coulomb friction is applied
// at the mechanism. ASSUMED model, published stall/free numbers.
constexpr double kStallNm[3] = {2.1, 1.05, 0.35};    // red, green, blue
constexpr double kFreeRpm[3] = {100.0, 200.0, 600.0};
constexpr double kTicksPerRev[3] = {1800.0, 900.0, 300.0};
constexpr double kTorqueLineGain = 2.0;
constexpr double kMotorFricFrac = 0.04;    // Coulomb, share of stall
constexpr double kMotorViscFrac = 0.08;    // viscous at free speed, share of stall
constexpr int kCmdLatencyMs = 10;          // brain -> motor
constexpr int kMotorSamplePeriodMs = 10;   // encoder telemetry
constexpr double kHoldKpVPerDeg = 2.0;     // HOLD brake position loop (output deg)
constexpr double kVelKpFrac = 6.0;         // move_velocity inner P loop (unused by this robot)

// ---- odometry sensors (truth)
constexpr double kTrackWheelDiaIn = 2.0;       // --wheel-dia 2.125 for the Omniwheel::NEW_2 scenario
constexpr double kTrackWheelOffsetIn = 0.364;  // + = right of centre. Final CAD: the pair of 2 in omnis is 0.364 in RIGHT (was -1.0,
                                               // the old code's guess, so sim truth and code agreed before and still do)
constexpr double kRotationCpr = 4096.0;
constexpr int kRotationPeriodMs = 10;
constexpr int kImuPeriodMs = 10;
constexpr int kImuCalMs = 2000;
// Each IMU has its own 10 ms sample clock. Noise seeds: random phase per sensor and +-kImuJitterMs
// arrival jitter, plus an occasional lost packet. --no-noise: both on phase 3, no jitter (the old,
// perfectly simultaneous sampling). Real V5 timing is NOT known; --imu-jitter MS overrides.
constexpr double kImuJitterMs = 1.0;
constexpr double kImuMissProb = 0.001;      // per sample, only when jitter is on

// ---- distance sensors
constexpr int kDistPeriodMs = 33;
constexpr double kDistConeHalfDeg = 10.0;   // ASSUMED 20 deg full cone, nearest of 5 rays. Vertically the same half angle:
                                            // an object lower than the sensor is seen once the cone has dropped to its top
constexpr int kDistRays = 5;
constexpr double kDistMaxMm = 2000.0;
constexpr double kFrontSensorHIn = 4.5, kSideSensorHIn = 6.6;   // sides as built (Austin's drawing, 19 Nov)
constexpr double kDistJitterMm = 3.0, kDistJitterFrac = 0.01;   // sample to sample
constexpr double kDistOffSdMm = 5.0, kDistScaleSd = 0.015;      // per sensor systematic (noise mode)
constexpr double kDistCalIn = 6.0;                              // biasIn calibration distance

// ---- DR4B lift (truth)
// Final CAD: stage 1 11.000 in (22 holes), stage 2 13.500 in (27 holes), hole to hole (were 11.495 / 14.5).
// Rest directions: stage 1 -128.7 with its lower bars sitting on the two 5.5 W motors (CAD range -129.75, if the Brain
// stops it first, to -128.7); stage 2 is NOT fixed by the CAD (-26.4 to -24.3, which 60T tooth is meshed), -25.0 is the
// code's value. Truth is these +-kLiftAngleErrDeg per seed (each stage), which covers both ranges. --lift-err DEG shifts both.
constexpr double kL1In = 11.0, kL2In = 13.5, kRest1Deg = -147.0, kRest2Deg = -32.17;   // 19 Nov CAD: on the crank axle standoff, stage 1 33 deg below level, stage 2 32.17
constexpr double kLiftAngleErrDeg = 2.0;
constexpr double kCrank = 4.0;              // motor deg per lift deg (12T -> 48T)
constexpr double kCarriageKg = 1.02;        // claw + rollers + half of stage 2, at the claw end. ASSUMED, +-10 % per seed
constexpr double kMidStageKg = 0.45;        // middle plates + stage 1 bars + half of stage 2, at the middle pivot. ASSUMED,
                                            // +-30 % per seed. Rises with stage 1 only (load peaks near 47 deg), which the
                                            // robot's gravityShare() feedforward does not know about.
constexpr double kObjectKg = 0.11;          // held object, ASSUMED (same for a bare Pin and a Cup+Pin: not measured)
constexpr double kLiftArmsInertia = 0.03;   // kg m^2 at the shaft, ASSUMED
constexpr double kLiftFricNm = 0.5;         // Coulomb at the lift shaft, ASSUMED, +-30 % per seed
// The owner's lift table (lift angle deg -> rise in, from motor degrees / 4 and height - 3.2). The CAD linkage is up to
// 1.4 in off it mid stroke, so the TRUE rise follows the table. Fore-aft travel still comes from the linkage.
constexpr double kLiftTableDeg[] = {0, 20, 40, 60, 80, 100, 103};
constexpr double kLiftTableIn[] = {0.0, 8.3, 17.0, 25.1, 31.7, 36.1, 36.5};
constexpr double kLiftBandNm = -1.0;        // below 0 = the band model below. --lift-bands NM puts back a constant assist (was 3.0, assumed)
// Rubber bands (Austin, 19 Nov): 6 x #64, 3 a side, from the tower to the lower stage 1 bar 9.0 in from its tower pivot
// (hole 5, the middle joint is hole 1). He gave the stretched length at both ends of the stroke: 12.0 in folded, 6.1 in
// at the top. kBandTowerFwdIn / UpIn (from the stage 1 tower pivot) is the one point near the tower that gives exactly
// those two lengths. Pull per band is TYPICAL, not measured: #64 is 3.5 in flat, 1/4 x 1/32 in, two strands, rubber
// shear modulus 0.45 MPa, so F = kBandN x (stretch - 1 / stretch^2): 15 N at 12 in, 6.4 N at 6.1 in. +-kBandSpread per
// seed. With the ASSUMED lift masses above the six bands carry 80 to 100 % of the empty lift's weight.
constexpr int kBandCount = 6;
constexpr double kBandBarIn = 9.0, kBandTowerFwdIn = 0.586, kBandTowerUpIn = 3.92, kBandFlatIn = 3.5, kBandN = 4.5, kBandSpread = 0.25;
constexpr double kLiftSlopDeg = 1.0;        // ASSUMED droop of the loaded lift below what the motor encoders say: 12T/48T and
                                            // shaft backlash, joint play, bar sag, all expressed at the lift shaft. The lift is
                                            // tared resting on its stop, so the encoders cannot see it (worst case: all of the
                                            // backlash is free at tare). 1 deg is about 0.4 in at the claw. 0 to 2 deg per
                                            // seed, --lift-slop DEG
constexpr double kKgTuneErr = 0.15;         // kG is "raise the volts until it holds": +- this share per seed
// Link drive (Austin, 19 and 23 Nov: a rigid 6.0 in link, it was a 5.918 in string), same numbers as dr4b:: in skills_support.hpp: crank axle from the stage 1 tower pivot,
// crank arm, tie hole from the pivot, string length, stage 1 angle on the stop.
constexpr double kCrankFwdIn = 0.250, kCrankDownIn = 6.750, kCrankArmIn = 2.061, kTieIn = 2.000, kLinkIn = 6.000, kStage1FoldedDeg = -33.0;
constexpr double kLiftTopDeg = 98.0;   // was 105.0 before the string: the stroke ends with stage 1 65 deg above level       // mechanical top stop. Final CAD, low confidence: stage 2's own bars meet at about
                                            // 105 deg (was 124, assumed). The code's kTopDeg is 100
constexpr double kSwayHz = 3.0, kSwayZeta = 0.15, kSwayFullRiseIn = 30.0;   // ASSUMED fore-aft sway of the raised claw
                                            // under chassis acceleration (1 mode, deflection grows with lift height)

// ---- Toggle spinner (final CAD, robot frame: forward, up, right of the FRAME centre) and the Toggle on the red wall
// Only the RIGHT side is assembled in the CAD: the right 5.5 W motor chain-drives the one paddle. The left motor has a
// sprocket but no chain, 18T or paddle, so in the sim it spins free (the robot code also treats the two as not linked).
constexpr double kPaddleAxisFwdIn = 7.628, kPaddleAxisUpIn = 13.004;   // 18T sprocket axis, on the right tower plates
constexpr double kPaddleRightLoIn = 5.38, kPaddleRightHiIn = 7.41;     // the paddle occupies this band (right side only)
constexpr double kPaddleLongIn = 1.847, kPaddleShortIn = 1.355;        // radius of its two ends about that axis
constexpr double kPaddleRatio = 18.0 / 8.0;      // motor turns per paddle turn (CAD tooth count; the CAD sprockets cannot mesh as drawn)
constexpr double kPaddleCadDeg = 9.2;            // long end direction in the CAD pose (0 = forward, 90 = up): tip at forward 9.451, up 13.3
constexpr double kPaddleParkDeg = 180.0;         // where the paddle starts: long end BACK, as skills_support.hpp tells the team to park it
constexpr double kPaddleParkErrDeg = 30.0;       // parked by hand, so +- this per seed (ASSUMED). Nothing in the code homes it. --paddle-deg DEG
constexpr double kToggleMotorStallNm = 0.5;      // 5.5 W motor: published 0.5 N m stall, 200 rpm free, 900 ticks/rev
constexpr double kPaddleJ = 1.0e-4;              // kg m^2 at the paddle shaft (paddle, 18T, chain). ASSUMED
constexpr double kToggleResistNm = 0.25;         // torque at the paddle shaft that turning the Toggle costs while an end is in it.
                                                 // ASSUMED (about 5 N at the tip); the one 5.5 W motor stalls at 1.1 N m there
constexpr double kMovePosKp = 10.0;              // move_relative: target speed = this x position error (1/s), capped at the
                                                 // commanded rpm. ASSUMED stand-in for the V5 firmware profile
// Robot parts at Toggle height that meet the Toggle before the rails meet the wall: {radius about the paddle axis, right}.
// Chain on the 18T (reaches forward 8.86) and the tower plates' rounded nose (8.13). Right side only, as in the CAD.
constexpr double kToggleBumpers[2][2] = {{1.235, 5.42}, {0.5, 4.30}};
// The Toggle: three-sided prism on top of the red wall, 25.8 in long, centred on the wall centre. Turning axis 1.246 in
// beyond the wall's inner face and 13.244 in up, circumradius 1.188, one edge toward the field at rest (0.06 in beyond the
// wall face). Spans up 12.22 to 14.27 at rest, 12.06 to 14.43 while turning.
constexpr double kToggleLenIn = 25.8, kToggleAxisBeyondIn = 1.246, kToggleAxisUpIn = 13.244, kToggleCircumRIn = 1.188;
constexpr double kToggleTopTurningIn = kToggleAxisUpIn + kToggleCircumRIn;   // 14.43
constexpr double kToggleClearIn = 1.0;           // how far over that the claw underside is taken for kToggleUp

// ---- roller claw
constexpr double kRollerInertia = 0.0008;   // kg m^2 at the motor output
constexpr double kClawReachIn = 8.0;        // frame centre to held-object centre, lift at rest = 6.5 ahead of the axle (derived from the CAD carriage).
                                            // The claw is not in the CAD: kept where the route expects it
constexpr double kClawHalfWIn = 2.6;        // claw body half width (pushes loose and standing objects)
constexpr double kRollerThickIn = 0.25;     // ASSUMED roller / side plate thickness inside that
constexpr double kThroatBackIn = 3.15;      // seated object centre to the claw sensor face
constexpr double kSeatedMm = 40.0;          // claw sensor reading with an object seated
constexpr double kGrabLatIn = kClawHalfWIn - 1.58 - kRollerThickIn;   // 0.77: a Cup (r 1.58) fits between the plates. --grab-lat
constexpr double kGrabKeepFrac = 0.5;       // share of the grab's sideways offset the held object keeps. ASSUMED
constexpr double kGrabAheadIn = 1.75;       // rollers reach this far ahead of the seated point. ASSUMED, --grab-ahead
constexpr double kGrabBehindIn = 0.75;      // object can be this far "inside" before it is pushed
constexpr double kGrabMaxRiseIn = 2.0;      // claw within this of pickup height still grips
constexpr double kTipPushIn = 1.0;          // a standing Cup pushed further than this tips over
constexpr double kPullInPerS = 15.0;        // rollers draw an object in at this speed at 12 V
constexpr int kReleaseMs = 80;              // outtake time before the object leaves the grip. ASSUMED, --release-ms
                                            // (the route waits 200 ms after rclaw::release(), the code outtakes for 300)
constexpr double kReleaseFwdIn = 0.5;       // forward carry of a released object. ASSUMED, --release-fwd

// ---- objects and Goals
constexpr double kCupR = 1.58, kCupH = 6.48, kCupPinH = 9.8, kFlatPinR = 1.58, kFlatPinH = 3.16;
constexpr double kGoalR = 3.22;
constexpr double kPitch = 3.32, kNest = 3.165, kObjLen = 6.49;
constexpr double kPitchOld = 3.55;          // the "older reading" skills_support.hpp mentions. Noise seeds draw the true
                                            // pitch from kPitch..kPitchOld, --no-noise uses kPitch, --pitch IN fixes it
constexpr double kGoalTop[3] = {5.77, 3.25, 8.77};     // Short, Alliance, Tall. Short and Tall confirmed by the owner 18 Nov (146.5 / 222.7 mm)
constexpr double kGoalTopDraw[3] = {5.77, 3.25, 8.77}; // --draw-heights
// Placement window: the released object's centre must be within kCaptureR of the Goal
// centre, and its bottom between kPlaceBelowTol below and kPlaceAboveMax above the TOP of
// what is already there (Goal base top if empty, else the top of the stack: seat +
// (objects-1)*pitch + 6.49, the same expression the distance beams use). An object lower
// than that cannot get over the stack: it arrives from the side and hits it.
// ASSUMED and not from the game manual: any object seats on any stack or bare base.
constexpr double kCaptureR = 1.0;           // ASSUMED, --capture-r
constexpr double kPlaceBelowTol = 0.15;
constexpr double kPlaceAboveMax = 2.5;
constexpr double kMidfieldHalfDiag = 23.40;
constexpr int kRefillMs = 1000;             // human loads the next Match Load within 1 s
constexpr double kLoaderX = 58.80, kLoaderY = 68.38;
constexpr double kLoaderFaceY = 66.47;      // chute face (3.74 deep from the wall)
constexpr double kLoaderHalfW = 2.42;       // chute 4.84 wide

// ---- noise (per seed; all off with --no-noise)
constexpr double kStartXYIn = 0.5, kStartDeg = 1.0;
constexpr double kStartGapIn = 0.15;        // rear to wall gap at the start: one-sided because the robot cannot be inside
                                            // the wall, small because it is pushed against it
constexpr double kImuNoiseDeg = 0.02, kImuDriftDegPerMinSd = 0.6, kImuGainSd = 0.003;
constexpr double kImuCommonGainSd = 0.01;   // both scales are the placeholder 1.0 in main.cpp and the old sensor needed
                                            // 1.0112, so the uncalibrated pair shares a gain error of about 1 %.
                                            // --imu-gain 1.0 is the "scales measured" scenario
constexpr double kWheelDiaErrMean = 0.0, kWheelDiaErrSd = 0.0075, kWheelOffsetSd = 0.25;
constexpr double kPreloadDzSdIn = 0.15;     // how high the preload sits in the claw relative to a floor pickup
constexpr double kMuSpread = 0.10, kRollSpread = 0.20, kMotorSpread = 0.05;
constexpr double kMountErrSdIn = 0.10;
constexpr double kLoaderJitterIn = 0.25;
}  // namespace simcfg
