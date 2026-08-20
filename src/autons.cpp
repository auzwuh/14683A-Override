#include "autons.hpp"

#include "macros.hpp"
#include "gen/chassis/chassis.hpp"
#include "gen/electronics.h"
#include "pros/rtos.hpp"

#include <cstdlib>

extern arc::Chassis chassis;
extern arc::MotorGroup intake;
extern arc::MotorGroup lift;
extern arc::MotorGroup claw;
extern arc::MotorGroup clawRot;



namespace {

using namespace robot::tune;

bool waitLift(int target, int timeoutMs) {
    const std::uint32_t deadline = pros::millis() + static_cast<std::uint32_t>(timeoutMs);
    while (pros::millis() < deadline) {
        if (std::abs(static_cast<int>(lift.get_position()) - target) <= LIFT_TOL) return true;
        pros::delay(10);
    }
    return false;
}

bool waitRot(int target, int timeoutMs) {
    const std::uint32_t deadline = pros::millis() + static_cast<std::uint32_t>(timeoutMs);
    while (pros::millis() < deadline) {
        if (std::abs(static_cast<int>(clawRot.get_position()) - target) <= ROT_TOL) return true;
        pros::delay(10);
    }
    return false;
}


void liftTo(int deg) {
    const bool descending = lift.get_position() > deg;
    lift.move_absolute(deg, descending ? LIFT_DOWN_VEL : LIFT_UP_VEL);
}


void present(int liftDeg) {
    liftTo(liftDeg);
    waitLift(LIFT_ROT_CLEAR, 800);
    clawRot.move_absolute(ROT_SCORE, ROT_VEL);
}

constexpr float SPD_PIN_IN  = 22.5f;   // ~10.3 in/s - threading into the cluster
constexpr float SPD_PIN_OUT = 41.0f;   // ~18.9 in/s - withdrawing with the Pin
constexpr float SPD_GOAL    = 45.0f;   // ~20.7 in/s - placing onto a loaded Goal

int scoreHeight(int pinsOnGoal) {
    return LIFT_SHORT + pinsOnGoal * PIN_NEST_DEG;
}

void stowArm() {
    clawRot.move_absolute(ROT_STOW, ROT_VEL);
    waitRot(ROT_STOW, 600);
    liftTo(LIFT_STOW);
}

//
//   x' = m00*x + m01*y
//   y' = m10*x + m11*y
//   theta' = mirror ? (c - theta) : (c + theta)
//
struct Frame {
    float m00, m01, m10, m11;
    bool mirror;
    float c;
    int rams;     // red needs TWO hits, blue ONE.
};

constexpr Frame kRedLeft   = { 1,  0,  0,  1, false,   0.0f, 2};
constexpr Frame kRedBottom = { 0, -1, -1,  0, true,  270.0f, 2};
constexpr Frame kBlueRight = {-1,  0,  0, -1, false, 180.0f, 1};
constexpr Frame kBlueTop   = { 0,  1,  1,  0, true,   90.0f, 1};

struct Pt {
    float x, y;
};

Pt map(const Frame& f, float x, float y) {
    return {f.m00 * x + f.m01 * y, f.m10 * x + f.m11 * y};
}

float mapTheta(const Frame& f, float theta) {
    const float t = f.mirror ? (f.c - theta) : (f.c + theta);
    return t < 0.0f ? t + 360.0f : (t >= 360.0f ? t - 360.0f : t);
}


void runQuadrant(const Frame& f) {
    const Pt start   = map(f,  0.00000f, 63.2057f);   // front flat on the wall
    const Pt ramHome = map(f,  0.00000f, 61.0000f);   // clear of the Toggle face
    const Pt ramHit  = map(f,  0.00000f, 64.7000f);   // commanded past the wall
    const Pt standby = map(f,  4.83000f, 48.0000f);   // clear of the Goal, free to pivot
    const Pt score   = map(f, 15.11000f, 48.0000f);   // aligner pressed on the Goal
    const Pt lead    = map(f,  7.91446f, 48.0000f);   // on the Goal axis, 40 deg ray
    const Pt pin     = map(f, 18.99269f, 34.7975f);   // cluster Pin, front-first
    const Pt finish  = map(f,  7.00000f, 48.0000f);   // peeled off, clear of perimeter

    chassis.setPose(start.x, start.y, mapTheta(f, 0.0f));

    liftTo(LIFT_TOGGLE);
    chassis.moveToPoint(ramHome.x, ramHome.y, {.timeout = 462, .forwards = false, .async = true});
    chassis.waitUntilDone();
    waitLift(LIFT_TOGGLE, 1200);

    for (int i = 0; i < f.rams; ++i) {
        // Target is past the wall on purpose: the drive stalls into the Toggle.
        chassis.moveToPoint(ramHit.x, ramHit.y, {.timeout = 598});
        pros::delay(400);
        chassis.moveToPoint(ramHome.x, ramHome.y, {.timeout = 518, .forwards = false});
        pros::delay(100);
    }
    chassis.turnToPoint(standby.x, standby.y, {.timeout = 431, .forwards = false});
    chassis.moveToPoint(standby.x, standby.y, {.timeout = 1002, .forwards = false});
    pros::delay(100);
    chassis.turnToPoint(score.x, score.y, {.timeout = 985, .forwards = false});
    chassis.moveToPoint(score.x, score.y,
                        {.timeout = 1500, .forwards = false, .maxSpeed = SPD_GOAL, .async = true});

    // WINCH: lift first, rotator only once it clears the frame.
    // ONE Pin is already nested on this Short Goal at the start of the Match.
    present(scoreHeight(1));
    chassis.waitUntilDone();
    // The aligner is pressed on the Goal now 
    claw.move_absolute(CLAW_OPEN, CLAW_VEL);
    pros::delay(350);

    chassis.moveToPoint(lead.x, lead.y, {.timeout = 833});
    pros::delay(100);
    chassis.turnToPoint(pin.x, pin.y, {.timeout = 1371});
    chassis.moveToPoint(pin.x, pin.y,
                        {.timeout = 3725, .maxSpeed = SPD_PIN_IN, .async = true});

    clawRot.move_absolute(ROT_STOW, ROT_VEL);
    chassis.waitUntil(7.755596f);
    intake.move(127);
    waitRot(ROT_STOW, 400);
    liftTo(LIFT_STOW);
    chassis.waitUntilDone();

    pros::delay(450);
    claw.move_absolute(CLAW_GRIP, CLAW_VEL);
    pros::delay(900);

    chassis.moveToPoint(lead.x, lead.y,
                        {.timeout = 2284, .forwards = false, .maxSpeed = SPD_PIN_OUT});
    pros::delay(100);
    chassis.turnToPoint(score.x, score.y, {.timeout = 1371, .forwards = false});
    chassis.moveToPoint(score.x, score.y,
                        {.timeout = 1217, .forwards = false, .maxSpeed = SPD_GOAL, .async = true});
    intake.move(0);
    // TWO Pins on the Goal now: the pre-placed yellow one and our preload.
    present(scoreHeight(2));
    chassis.waitUntilDone();
    pros::delay(550);
    claw.move_absolute(CLAW_OPEN, CLAW_VEL);
    pros::delay(350);

    chassis.moveToPoint(finish.x, finish.y, {.timeout = 885});
    stowArm();
}

}

void Auton::overrideRedLeft()   { runQuadrant(kRedLeft); }
void Auton::overrideRedBottom() { runQuadrant(kRedBottom); }
void Auton::overrideBlueRight() { runQuadrant(kBlueRight); }
void Auton::overrideBlueTop()   { runQuadrant(kBlueTop); }
