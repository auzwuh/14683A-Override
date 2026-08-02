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

} 

void Auton::overrideAwpRed() {
    chassis.setPose(-42.000f, 64.590f, 180.00f);
    chassis.moveToPoint(-42.000f, 54.000f, {.timeout = 720, .errorExit = 4.7f, .minSpeed = 62.0f, .settle = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.moveToPose(-36.333f, 52.110f, 108.43f, {.timeout = 625, .dLead = 2.389f});
    pros::delay(590);
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.turnToPoint(-0.000f, 54.000f, {.timeout = 595});
    // pure-pursuit path 'override_awp_red_seg2_path_txt' expanded to 8 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(-36.333f, 52.110f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.moveToPoint(-34.433f, 42.314f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 107.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-27.625f, 35.648f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 113.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-17.832f, 33.764f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 126.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-7.923f, 34.534f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 104.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(0.091f, 40.175f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 51.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-0.000f, 50.094f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-0.000f, 54.000f, {.timeout = 275, .errorExit = 2.0f, .maxSpeed = 127.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-0.000f, 64.700f, {.timeout = 836});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(300);
    intake.move(127);
    pros::delay(800);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(350);
    chassis.moveToPoint(-0.000f, 48.000f, {.timeout = 904, .forwards = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(11.000f, 48.000f, {.timeout = 854, .forwards = false});
    chassis.moveToPoint(11.000f, 48.000f, {.timeout = 847, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(-2.000f, 48.000f, {.timeout = 922, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-35.315f, 0.047f, {.timeout = 458});
    // pure-pursuit path 'override_awp_red_seg7_path_txt' expanded to 8 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(-2.000f, 48.000f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-9.619f, 41.523f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-17.063f, 34.847f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-24.221f, 27.865f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-31.342f, 20.845f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-37.671f, 13.127f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 99.0f, .minSpeed = 30.0f, .settle = false});
    intake.move(127);
    chassis.moveToPoint(-37.092f, 3.854f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 102.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-35.315f, 0.047f, {.timeout = 276, .errorExit = 2.0f, .maxSpeed = 127.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.turnToPoint(-29.226f, -10.294f, {.timeout = 266});
    chassis.moveToPoint(-29.226f, -10.294f, {.timeout = 885});
    pros::delay(450);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(350);
    chassis.turnToPoint(-36.580f, -17.789f, {.timeout = 722});
    chassis.moveToPose(-36.580f, -17.789f, 241.46f, {.timeout = 832, .dLead = 4.200f, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    pros::delay(660);
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.turnToPoint(-54.000f, -0.000f, {.timeout = 985});
    // pure-pursuit path 'override_awp_red_seg10_path_txt' expanded to 4 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(-36.580f, -17.789f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 78.0f, .minSpeed = 30.0f, .settle = false});
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.moveToPoint(-38.021f, -7.959f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 52.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-44.747f, -0.681f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 47.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-54.000f, -0.000f, {.timeout = 416, .errorExit = 2.0f, .maxSpeed = 90.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-64.700f, -0.000f, {.timeout = 836});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(300);
    chassis.moveToPoint(-46.000f, -0.000f, {.timeout = 959, .forwards = false});
}

void Auton::overrideAwpBlue() {
    chassis.setPose(42.000f, -64.590f, 0.00f);
    chassis.moveToPoint(42.000f, -54.000f, {.timeout = 720, .errorExit = 4.24f, .minSpeed = 55.0f, .settle = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.moveToPose(36.333f, -52.110f, 288.43f, {.timeout = 625, .dLead = 2.389f});
    pros::delay(590);
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.turnToPoint(-0.000f, -54.000f, {.timeout = 595});
    // pure-pursuit path 'override_awp_blue_seg2_path_txt' expanded to 8 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(36.333f, -52.110f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.moveToPoint(34.433f, -42.314f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 107.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(27.625f, -35.648f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 113.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(17.832f, -33.764f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 126.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(7.923f, -34.534f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 104.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-0.091f, -40.175f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 51.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-0.000f, -50.094f, {.timeout = 275, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(-0.000f, -54.000f, {.timeout = 275, .errorExit = 2.0f, .maxSpeed = 127.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-0.000f, -64.700f, {.timeout = 836});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(300);
    intake.move(127);
    pros::delay(800);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(350);
    chassis.moveToPoint(-0.000f, -48.000f, {.timeout = 904, .forwards = false, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(-11.000f, -48.000f, {.timeout = 854, .forwards = false});
    chassis.moveToPoint(-11.000f, -48.000f, {.timeout = 847, .forwards = false});
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.moveToPoint(2.000f, -48.000f, {.timeout = 922, .async = true});
    chassis.waitUntil(0.000f);
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.waitUntilDone();
    chassis.turnToPoint(35.315f, -0.047f, {.timeout = 458});
    // pure-pursuit path 'override_awp_blue_seg7_path_txt' expanded to 8 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(2.000f, -48.000f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(9.619f, -41.523f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(17.063f, -34.847f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(24.221f, -27.865f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(31.342f, -20.845f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 127.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(37.671f, -13.127f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 99.0f, .minSpeed = 30.0f, .settle = false});
    intake.move(127);
    chassis.moveToPoint(37.092f, -3.854f, {.timeout = 276, .errorExit = 3.0f, .maxSpeed = 102.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(35.315f, -0.047f, {.timeout = 276, .errorExit = 2.0f, .maxSpeed = 127.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.turnToPoint(29.226f, 10.294f, {.timeout = 266});
    chassis.moveToPoint(29.226f, 10.294f, {.timeout = 885});
    pros::delay(450);
    intake.move(0);
    claw.move_absolute(CLAW_GRIP, 100);
    pros::delay(350);
    chassis.turnToPoint(36.580f, 17.789f, {.timeout = 722});
    chassis.moveToPose(36.580f, 17.789f, 61.46f, {.timeout = 832, .dLead = 4.200f, .async = true});
    chassis.waitUntil(0.000f);
    lift.move_absolute(310, 100);
    clawRot.move_absolute(160, 100);
    chassis.waitUntilDone();
    pros::delay(660);
    claw.move_absolute(CLAW_OPEN, 100);
    pros::delay(300);
    chassis.turnToPoint(54.000f, -0.000f, {.timeout = 985});
    // pure-pursuit path 'override_awp_blue_seg10_path_txt' expanded to 4 waypoints (no static/ asset pipeline)
    chassis.moveToPoint(36.580f, 17.789f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 78.0f, .minSpeed = 30.0f, .settle = false});
    clawRot.move_absolute(0, 100);
    lift.move_absolute(0, 100);
    chassis.moveToPoint(38.021f, 7.959f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 52.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(44.747f, 0.681f, {.timeout = 416, .errorExit = 3.0f, .maxSpeed = 47.0f, .minSpeed = 30.0f, .settle = false});
    chassis.moveToPoint(54.000f, -0.000f, {.timeout = 416, .errorExit = 2.0f, .maxSpeed = 90.0f, .minSpeed = 45.0f, .settle = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(64.700f, -0.000f, {.timeout = 836});
    // localizer.applyImmediateCorrectionAuto();   // no Atticus localizer on this robot
    pros::delay(300);
    chassis.moveToPoint(46.000f, -0.000f, {.timeout = 959, .forwards = false});
}

