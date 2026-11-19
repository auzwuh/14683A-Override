#pragma once
// Self calibration. The robot measures its own sensor constants against a field wall, saves
// them on the microSD card, and every later start of the program loads them by itself.
//
// What it sets
//   IMU scale A and B      two full turns between two back squares on the same wall
//   tracking wheel offset  how far the wheel rolls while the robot turns on the spot (same two turns)
//   distance sensor biasIn each sensor is turned to face that wall from a known distance
//   claw sensor            set from the preload at the start of every autonomous run (clawFromPreload)
// What it can NOT set
//   tracking wheel size    stays at the value in main.cpp (2.0 in). Nothing on the robot knows a true
//                          distance better than that wheel does.
//
// How to run it (about 15 s, once per robot build, again after moving a sensor)
//   1. Lift down. Back bumper against a field wall. The robot only moves 3.5 in off the wall and
//      turns there, so it needs 12 in of clear tiles all round its middle and 8 in of bare wall
//      to each side. The skills start position is clear enough, with the field set up.
//   2. Driver control, NOT plugged into field control. Hold LEFT and RIGHT on the controller
//      for half a second. The robot drives itself and ends back on the wall.
//   3. One short buzz = saved. Three long ones = something was out of range, read the terminal.
// Without a microSD card the values are used until the program is closed and are printed, so
// they can be typed into main.cpp (kImuScaleA / B) and skills_support.hpp (mounts biasIn).
//
// Why a wall. A robot standing still cannot tell how far off its gyro scale is or where a
// distance sensor's zero is. Both need a known turn and a known distance. Squaring the back
// bumper on a wall gives an exact heading twice and an exact distance to that wall.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "dual_imu.hpp"
#include "skills_support.hpp"

extern arc::DualIMU imu;
extern arc::TrackingWheel verticalTrackingWheel;

namespace calib {
inline const char* kFile = "/usd/override_cal.txt";
inline double kOutIn = 3.5;         // how far off the wall the robot turns. The back corners need 2.4 in. Kept small so the
                                    // claw and preload (11 in out) stay clear of the Goal next to the skills start
inline int kDriveMv = 4000;         // creep off the wall. A 3 in move is too short for the drive PID
inline int kSquareMinMs = 350, kSquareMaxMs = 2500;   // back square push: until the robot has stopped moving and turning
inline double kLiftDeg = 45.0;       // lift angle while calibrating: its back end is then 2 in above the wall top
inline double kLiftClearIn = 6.0;    // pull this far off the wall before the lift goes up or down
inline int kTurns = 2;              // full turns for the IMU scale. 2 is enough: the squares are good to 0.3 deg in 720
inline double kMaxBiasIn = 1.5;     // a bias bigger than this means something else was in the beam
inline double kFlatIn = 0.25;       // after a square the tracking wheel must be within this of where the first square left it
inline double kClawMarginMm = 25.0; // claw sensor counts as "object in the claw" up to the preload reading plus this

inline double imuScale[2] = {NAN, NAN};   // NAN = never measured, main.cpp's value is used
inline double wheelOffset = NAN;          // tracking wheel, in right of the turning centre. NAN = main.cpp's value
inline bool loaded = false;

inline void apply() {
    for (int i = 0; i < 2; ++i)
        if (std::isfinite(imuScale[i])) imu.setScale(i, imuScale[i]);
    if (std::isfinite(wheelOffset)) verticalTrackingWheel.setOffset(static_cast<float>(wheelOffset));
}

// Called from initialize(). True when a saved calibration was found.
inline bool load() {
    std::FILE* f = std::fopen(kFile, "r");
    if (!f) {
        std::printf("[cal] no saved calibration (%s). IMU scales and sensor offsets are whatever the code has\n", kFile);
        return false;
    }
    char key[32];
    double v;
    while (std::fscanf(f, "%31s %lf", key, &v) == 2) {
        const std::string k = key;
        if (k == "imuA") imuScale[0] = v;
        else if (k == "imuB") imuScale[1] = v;
        else if (k == "wheelOffset") wheelOffset = v;
        else if (k == "biasFront") localizer.mounts[0].biasIn = v;
        else if (k == "biasLeft") localizer.mounts[1].biasIn = v;
        else if (k == "biasRight") localizer.mounts[2].biasIn = v;
    }
    std::fclose(f);
    apply();
    loaded = true;
    std::printf("[cal] loaded: IMU scale %.4f %.4f, tracking wheel offset %.3f in, distance bias front %.2f left %.2f right %.2f in\n",
                imuScale[0], imuScale[1], wheelOffset, localizer.mounts[0].biasIn, localizer.mounts[1].biasIn, localizer.mounts[2].biasIn);
    return true;
}

inline bool save() {
    std::FILE* f = std::fopen(kFile, "w");
    if (!f) return false;
    if (std::isfinite(imuScale[0])) std::fprintf(f, "imuA %.6f\n", imuScale[0]);
    if (std::isfinite(imuScale[1])) std::fprintf(f, "imuB %.6f\n", imuScale[1]);
    if (std::isfinite(wheelOffset)) std::fprintf(f, "wheelOffset %.4f\n", wheelOffset);
    const char* name[3] = {"biasFront", "biasLeft", "biasRight"};
    for (int i = 0; i < 3; ++i)
        if (std::isfinite(localizer.mounts[i].biasIn)) std::fprintf(f, "%s %.4f\n", name[i], localizer.mounts[i].biasIn);
    std::fclose(f);
    return true;
}

// Claw sensor, from the preload. Call at the very start of autonomous(), with the preload in
// the claw. Only when no value is set in the code (kSeesObjectMm is NAN).
// "Object in the claw" = a reading under the preload's reading plus kClawMarginMm. Half a second
// after the preload has been released a background task reads the EMPTY claw as a check. If the
// empty claw reads less than 10 mm more than the preload did, the sensor cannot tell the two
// apart and the value is taken out again (grabs are then not checked, as before).
inline void clawFromPreload() {
    if (!std::isnan(rclaw::kSeesObjectMm)) return;
    int r[3];
    for (int i = 0; i < 3; ++i) { r[i] = rclaw::sensor().get(); pros::delay(20); }   // 3 readings, 60 ms
    std::sort(r, r + 3);
    const int mm = r[1];
    if (!(mm > 0 && mm < 150)) {
        std::printf("[cal] claw sensor reads %d mm, no preload seen. Grabs are not checked this run\n", mm);
        return;
    }
    rclaw::kSeesObjectMm = mm + kClawMarginMm;
    std::printf("[cal] claw sensor reads %d mm with the preload, object = under %.0f mm\n", mm, rclaw::kSeesObjectMm);
    pros::Task([mm] {
        const auto t0 = pros::millis();
        while (skills::holding && pros::millis() - t0 < 8000) pros::delay(20);   // until the preload is released
        if (skills::holding) return;
        pros::delay(500);
        const int empty = rclaw::sensor().get();
        if (empty > 0 && empty < mm + 10) {
            rclaw::kSeesObjectMm = NAN;
            std::printf("[cal] empty claw reads %d mm, the preload read %d. Too close to tell apart, grabs are not checked\n", empty, mm);
        } else if (empty > 0 && empty < rclaw::kSeesObjectMm) {
            rclaw::kSeesObjectMm = 0.5 * (mm + empty);
            std::printf("[cal] empty claw reads %d mm, object = under %.0f mm\n", empty, rclaw::kSeesObjectMm);
        }
    }, "claw-check");
}

namespace detail {
// Push the back bumper on the wall until the robot has stopped rolling AND stopped turning for
// 0.25 s. After two turns on an unscaled IMU the robot comes back up to 10 deg off square, and
// a fixed 0.45 s push left it short of square (or short of the wall) in the simulator.
inline void square() {
    localizer.beginSquare(true);   // stops tracking and any motion, pushes the back bumper
    const auto t0 = pros::millis();
    double lastRot = imu.rawRotation(0);
    arc::Pose lastPose = chassis.getPose();
    auto stillSince = t0;
    while (pros::millis() - t0 < static_cast<std::uint32_t>(kSquareMaxMs)) {
        pros::delay(10);
        const double r = imu.rawRotation(0);
        const arc::Pose p = chassis.getPose();
        if (std::fabs(r - lastRot) > 0.03 || std::hypot(p.x - lastPose.x, p.y - lastPose.y) > 0.02) {
            lastRot = r;
            lastPose = p;
            stillSince = pros::millis();
        }
        if (pros::millis() - t0 > static_cast<std::uint32_t>(kSquareMinMs) && pros::millis() - stillSince > 250) break;
    }
    leftDrive.move_voltage(0);
    rightDrive.move_voltage(0);
    pros::delay(150);
}
// Straight off the wall until the odometry says the robot origin is yIn from it.
inline void driveOut(double yIn) {
    const auto t0 = pros::millis();
    leftDrive.move_voltage(kDriveMv);
    rightDrive.move_voltage(kDriveMv);
    while (chassis.getPose().y < yIn - 0.3 && pros::millis() - t0 < 1500) pros::delay(5);
    leftDrive.brake();
    rightDrive.brake();
    pros::delay(250);
}
// The drive coasts when stopped (main.cpp). For this routine it brakes, so a 3.5 in creep is 3.5 in.
inline void brakeMode(bool on) {
    const auto m = on ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST;
    leftDrive.set_brake_mode_all(m);
    rightDrive.set_brake_mode_all(m);
}
}  // namespace detail

// The whole routine. Returns true when every value came out inside its limits. When it returns
// false nothing was saved, and whatever could not be measured keeps the value it had before.
inline bool run() {
    const double back = localizer.kBackBumperIn;   // every square in here is on the FRAME: the lift is raised over the wall first
    if (std::isnan(back)) { std::printf("[cal] kBackBumperIn is not set\n"); return false; }
    std::printf("[cal] start\n");
    const double oldScale[2] = {imuScale[0], imuScale[1]};
    for (int i = 0; i < 2; ++i) imu.setScale(i, 1.0);   // measure the raw sensors
    detail::brakeMode(true);
    auto liftDown = [&] {
        detail::driveOut(chassis.getPose().y + kLiftClearIn);
        dr4b::to(dr4b::kLoader);
        dr4b::waitSettled(2500);
        dr4b::target() = NAN;   // back to the driver
    };
    auto fail = [&](const char* why) {
        liftDown();
        std::printf("[cal] FAILED, %s. Nothing saved\n", why);
        imuScale[0] = oldScale[0];
        imuScale[1] = oldScale[1];
        for (int i = 0; i < 2; ++i) imu.setScale(i, std::isfinite(imuScale[i]) ? imuScale[i] : 1.0);
        localizer.pauseContinuous(false);
        detail::brakeMode(false);
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        return false;
    };
    // The folded lift sticks out 1.7 in past the back of the frame and its end is only 8.5 in wide (the
    // frame is 13.6). Pushed on that, a skewed robot does not turn flat: simulated, 1 run in 8 was refused
    // and the others measured worse. So pull off the wall, raise the lift until its back end is above the
    // wall top, and every square below is on the frame. On the way down the back of the lift swings out
    // to 12 in behind the origin, so liftDown() pulls off the wall first.
    detail::driveOut(chassis.getPose().y + kLiftClearIn);
    dr4b::to(kLiftDeg);
    if (!dr4b::waitSettled(2500) || !(dr4b::rearIn(dr4b::angle()) == 0.0)) return fail("the lift did not get above the wall top");
    // Square on the wall and call that spot heading 0 (facing away from the wall), y = back.
    // flat: after a push, the tracking wheel has to be back where the first square left it. A robot
    // jammed on one corner stops short (0.6 in at 5 deg), and its turn count would be wrong.
    auto squareHome = [&](bool first) {
        detail::square();
        const bool flat = first || std::fabs(chassis.getPose().y - back) < kFlatIn;
        chassis.setPose(0, back, 0);
        return flat;
    };
    // n full turns clockwise between two squares, in steps of 120 deg (arc takes the short way,
    // so each step is +120). Returns false if the robot did not come back flat on the wall.
    double raw[2], rolled = 0.0;
    auto turns = [&](int n) {
        for (int i = 0; i < 2; ++i) raw[i] = imu.rawRotation(i);
        rolled = verticalTrackingWheel.getDistanceTraveled();
        detail::driveOut(back + kOutIn);
        for (int k = 1; k <= 3 * n; ++k) chassis.turnToHeading(120.0f * (k % 3), {.timeout = 1200});
        if (!squareHome(false)) return false;
        for (int i = 0; i < 2; ++i) raw[i] = 360.0 * n / (imu.rawRotation(i) - raw[i]);   // raw[] now holds the scale
        // Wall to wall the robot has gone nowhere, so everything the tracking wheel rolled it
        // rolled because it sits off the turning centre: rolled = -offset x angle (arc's odometry).
        rolled = -(verticalTrackingWheel.getDistanceTraveled() - rolled) / (2.0 * M_PI * n);   // rolled now holds the offset
        return true;
    };

    squareHome(true);
    // 1. Rough scale from ONE turn. An IMU 1 % off brings the robot back 3.6 deg off square, which
    //    the push still flattens. After two turns it would be 7 deg and more, and it can jam.
    if (!turns(1)) return fail("the robot did not come back flat on the wall after one turn");
    for (int i = 0; i < 2; ++i) {
        if (!(std::isfinite(raw[i]) && std::fabs(std::fabs(raw[i]) - 1.0) < 0.08)) return fail("an IMU is more than 8 % off, or not turning with the robot");
        imu.setScale(i, raw[i]);
    }
    const double rough[2] = {raw[0], raw[1]};
    // 2. Fine scale from kTurns turns with the rough scale in use (the robot now comes back square).
    //    setScale() only changes what the chassis sees, rawRotation() stays raw.
    if (!turns(kTurns)) return fail("the robot did not come back flat on the wall after the second set of turns");
    for (int i = 0; i < 2; ++i) {
        if (!(std::isfinite(raw[i]) && std::fabs(raw[i] / rough[i] - 1.0) < 0.006)) return fail("the two IMU scale measurements disagree");
        imuScale[i] = raw[i];
    }
    const double oldOffset = wheelOffset;
    if (std::isfinite(rolled) && std::fabs(rolled - verticalTrackingWheel.getOffset()) < 1.0) wheelOffset = rolled;
    else std::printf("[cal] tracking wheel offset came out %.2f in, not used\n", rolled);
    apply();

    // 3. Each distance sensor faces the same wall, straight after a fresh square and one turn, so
    //    the distance to the wall is the 3.5 in the tracking wheel just measured.
    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        DistanceMount& m = localizer.mounts[i];
        if (!m.dev || std::isnan(m.rightIn) || std::isnan(m.forwardIn)) continue;
        detail::driveOut(back + kOutIn);
        chassis.turnToHeading(static_cast<float>(std::remainder(180.0 - m.angleDeg, 360.0)), {.timeout = 1500});
        pros::delay(250);
        // 32 readings (1 s). One reading is good to about 0.15 in, and the front sensor's offset sets the
        // gap at the Toggle, where the paddle's window is 0.5 in wide
        const double read = 0.5 * (localizer.medianIn(i, 16) + localizer.medianIn(i, 16));
        const arc::Pose p = chassis.getPose();
        const double th = p.theta * M_PI / 180.0, beam = (p.theta + m.angleDeg) * M_PI / 180.0;
        const double ys = p.y + m.forwardIn * std::cos(th) - m.rightIn * std::sin(th);   // sensor's distance from the wall
        const double pred = ys / -std::cos(beam);
        const double bias = pred - read;
        if (std::isfinite(read) && std::cos(beam) < -0.9 && std::fabs(bias) <= kMaxBiasIn) {
            m.biasIn = bias;
        } else {
            ok = false;
            std::printf("[cal] distance sensor %d read %.2f in, expected %.2f, not used\n", i, read, pred);
        }
        chassis.turnToHeading(0, {.timeout = 1500});
        if (!squareHome(false)) ok = false;   // also leaves the robot where it started
    }
    liftDown();
    detail::square();   // back on the wall with the lift folded, as it started
    localizer.pauseContinuous(false);
    detail::brakeMode(false);

    const bool saved = ok && save();
    (void)oldOffset;
    std::printf("[cal] IMU scale A %.4f B %.4f | tracking wheel offset %.3f in | distance bias front %.2f left %.2f right %.2f in | %s, %s\n",
                imuScale[0], imuScale[1], wheelOffset, localizer.mounts[0].biasIn, localizer.mounts[1].biasIn, localizer.mounts[2].biasIn,
                ok ? "all in range" : "A DISTANCE SENSOR WAS OUT OF RANGE", saved ? "saved to the SD card" : "NOT SAVED (no SD card, or not all in range)");
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ok && saved ? "." : "---");
    return ok;
}
}  // namespace calib
