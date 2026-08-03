#include "autons.hpp"

#include "gen/chassis/chassis.hpp"
#include "gen/electronics.h"
#include "pros/rtos.hpp"

// Defined in main.cpp alongside the rest of the User Template.
extern gen::Chassis chassis;
extern gen::MotorGroup intake;
extern gen::MotorGroup lift;
extern gen::MotorGroup claw;
extern gen::MotorGroup clawRot;

namespace {

// ---------------------------------------------------------------------------
// TUNE THESE ON THE REAL ROBOT.  They are placeholders carried over from the
// ATTICUS mechanism presets and have never been measured against hardware.
// Every one is a motor position in degrees.
// ---------------------------------------------------------------------------
constexpr int CLAW_GRIP = 0;      // claw closed on a Pin
constexpr int CLAW_OPEN = -75;    // claw released

// lift: 0 = stowed, 310 = Short Goal height, 470 = Tall Goal, 700 = Toggle
// clawRot: 0 = stowed, 160 = scoring, 300 = flipped

}  // namespace

void Auton::overrideAwpRed() {
    chassis.setPose(-36.000f, 64.590f, 180.00f);
    chassis.moveToPoint(-36.000f, 59.000f, {.timeout = 636, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-5.110f, 48.000f, {.timeout = 1022});
    // pure-pursuit path 'override_awp_red_seg1_path_txt' expanded to 5 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(-36.000f, 59.000f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 88.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-26.074f, 60.164f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 78.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-16.492f, 57.926f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 62.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-11.266f, 49.783f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 43.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-5.110f, 48.000f, {.timeout = 499, .errorExit = 2.0f, .maxSpeed = 64.0f, .minSpeed = 45.0f, .settle = false});
    chassis.turnToPoint(-15.110f, 48.000f, {.timeout = 430, .forwards = false});
    chassis.moveToPoint(-15.110f, 48.000f, {.timeout = 982, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(-9.500f, 48.000f, {.timeout = 736, .errorExit = 3.84f, .minSpeed = 81.0f, .settle = false, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(54.000f, -0.000f, {.timeout = 527, .errorExit = 2.25f, .direction = gen::AngularDirection::CCW_COUNTERCLOCKWISE, .lockedSide = gen::LockedSide::LEFT, .minSpeed = 40.0f});
    chassis.moveToPoint(-0.000f, 54.000f, {.timeout = 742, .errorExit = 5.06f, .minSpeed = 73.0f, .settle = false});
    chassis.moveToPoint(-0.000f, 64.700f, {.timeout = 880});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(250);
    intake.move(127);
    chassis.moveToPoint(-0.000f, 61.000f, {.timeout = 518, .forwards = false});
    chassis.moveToPoint(-0.000f, 64.700f, {.timeout = 518});
    pros::delay(250);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(1000);
    chassis.moveToPoint(-0.000f, 48.000f, {.timeout = 1099, .forwards = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(15.110f, 48.000f, {.timeout = 1120, .forwards = false});
    chassis.moveToPoint(15.110f, 48.000f, {.timeout = 1211, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(6.000f, 48.000f, {.timeout = 937, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-54.000f, -0.000f, {.timeout = 645});
    // pure-pursuit path 'override_awp_red_seg11_path_txt' expanded to 10 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(6.000f, 48.000f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-1.307f, 41.177f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-9.342f, 35.248f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 90.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-18.991f, 33.804f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-28.886f, 35.068f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 63.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-34.734f, 28.087f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 99.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-35.929f, 18.160f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 98.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-40.274f, 9.413f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 96.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-48.253f, 3.409f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-54.000f, -0.000f, {.timeout = 387, .errorExit = 2.0f, .maxSpeed = 105.0f, .minSpeed = 45.0f, .settle = false});
    chassis.moveToPoint(-64.700f, -0.000f, {.timeout = 880});
    pros::delay(250);
    chassis.moveToPoint(-61.000f, -0.000f, {.timeout = 518, .forwards = false});
    chassis.moveToPoint(-64.700f, -0.000f, {.timeout = 518});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(250);
    chassis.moveToPoint(-48.000f, -0.000f, {.timeout = 1099, .forwards = false});
}

void Auton::overrideAwpBlue() {
    chassis.setPose(36.000f, -64.590f, 0.00f);
    chassis.moveToPoint(36.000f, -59.000f, {.timeout = 636, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(5.110f, -48.000f, {.timeout = 1022});
    // pure-pursuit path 'override_awp_blue_seg1_path_txt' expanded to 5 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(36.000f, -59.000f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 88.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(26.074f, -60.164f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 78.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(16.492f, -57.926f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 62.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(11.266f, -49.783f, {.timeout = 499, .errorExit = 3.0f, .maxSpeed = 43.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(5.110f, -48.000f, {.timeout = 499, .errorExit = 2.0f, .maxSpeed = 64.0f, .minSpeed = 45.0f, .settle = false});
    chassis.turnToPoint(15.110f, -48.000f, {.timeout = 430, .forwards = false});
    chassis.moveToPoint(15.110f, -48.000f, {.timeout = 982, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(9.500f, -48.000f, {.timeout = 736, .errorExit = 3.84f, .minSpeed = 81.0f, .settle = false, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-54.000f, -0.000f, {.timeout = 527, .errorExit = 2.25f, .direction = gen::AngularDirection::CCW_COUNTERCLOCKWISE, .lockedSide = gen::LockedSide::LEFT, .minSpeed = 40.0f});
    chassis.moveToPoint(-0.000f, -54.000f, {.timeout = 742, .errorExit = 5.06f, .minSpeed = 73.0f, .settle = false});
    chassis.moveToPoint(-0.000f, -64.700f, {.timeout = 880});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(250);
    intake.move(127);
    chassis.moveToPoint(-0.000f, -61.000f, {.timeout = 518, .forwards = false});
    chassis.moveToPoint(-0.000f, -64.700f, {.timeout = 518});
    pros::delay(250);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(1000);
    chassis.moveToPoint(-0.000f, -48.000f, {.timeout = 1099, .forwards = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-15.110f, -48.000f, {.timeout = 1120, .forwards = false});
    chassis.moveToPoint(-15.110f, -48.000f, {.timeout = 1211, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(-6.000f, -48.000f, {.timeout = 937, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(54.000f, -0.000f, {.timeout = 645});
    // pure-pursuit path 'override_awp_blue_seg11_path_txt' expanded to 10 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(-6.000f, -48.000f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(1.307f, -41.177f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(9.342f, -35.248f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 90.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(18.991f, -33.804f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(28.886f, -35.068f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 63.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(34.734f, -28.087f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 99.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(35.929f, -18.160f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 98.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(40.274f, -9.413f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 96.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(48.253f, -3.409f, {.timeout = 387, .errorExit = 3.0f, .maxSpeed = 105.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(54.000f, -0.000f, {.timeout = 387, .errorExit = 2.0f, .maxSpeed = 105.0f, .minSpeed = 45.0f, .settle = false});
    chassis.moveToPoint(64.700f, -0.000f, {.timeout = 880});
    pros::delay(250);
    chassis.moveToPoint(61.000f, -0.000f, {.timeout = 518, .forwards = false});
    chassis.moveToPoint(64.700f, -0.000f, {.timeout = 518});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(250);
    chassis.moveToPoint(48.000f, -0.000f, {.timeout = 1099, .forwards = false});
}

