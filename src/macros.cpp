#include "macros.hpp"

#include "gen/electronics.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

extern arc::Controller controller;
extern arc::MotorGroup intake;
extern arc::MotorGroup lift;
extern arc::MotorGroup pinRollers;
extern arc::MotorGroup clawRot;

namespace robot {

Mechanism mech;

using Button = arc::Controller::Button;

namespace {

int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} 

void Mechanism::init() {
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    clawRot.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    pinRollers.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    lift.tare_position_all();
    clawRot.tare_position_all();

    stage_ = 0;
    r2HeldSince_ = 0;
    r2ResetFired_ = false;
}

void Mechanism::driveLift() {
    const int target = clampInt(tune::LIFT_STAGE_DEG[stage_], tune::LIFT_MIN_DEG, tune::LIFT_MAX_DEG);
    const int current = static_cast<int>(lift.get_position());
    const int error = target - current;
    const int output = clampInt(static_cast<int>(tune::LIFT_KP * error), -127, 127);
    lift.move(output);
}

void Mechanism::driveRotatorAndRollers() {
    // --- intake path: front intake and mechanically linked Pin rollers ---
    const int rollerOutput = rollerCommand(controller.holding(Button::L1),
                                            controller.holding(Button::R1));
    intake.move(rollerOutput);
    pinRollers.move(rollerOutput);

    // --- rotator: auto-preps for scoring, manual jog overrides it live ---
    int rotDir = 0;
    if (controller.holding(Button::Right)) rotDir = 1;
    else if (controller.holding(Button::Y)) rotDir = -1;

    const int rotPos = static_cast<int>(clawRot.get_position());
    if (rotDir > 0 && (rotPos >= tune::ROT_MAX || lift.get_position() < tune::LIFT_ROT_CLEAR)) rotDir = 0;
    if (rotDir < 0 && rotPos <= tune::ROT_MIN) rotDir = 0;

    if (rotDir != 0) {
        clawRot.move(rotDir * tune::ROT_JOG_CMD);
    } else {
        driveRotatorAuto();
    }
}

void Mechanism::driveRotatorAuto() {
    const bool prepToScore = stage_ != 0 && lift.get_position() >= tune::LIFT_ROT_CLEAR;
    clawRot.move_absolute(prepToScore ? tune::ROT_SCORE_DEG : tune::ROT_STOW_DEG, tune::ROT_VEL);
}

void Mechanism::setStage(int s) {
    stage_ = clampInt(s, 0, tune::LIFT_STAGE_COUNT - 1);
}

void Mechanism::tickAuton() {
    driveLift();
    driveRotatorAuto();
}

//
//   L1  hold        intake in
//   R1  hold        outtake
//   L2  tap         cascade: step UP one stage (tap 3x to reach stage 3)
//   R2  tap         cascade: step DOWN one stage
//   R2  hold ~1s    cascade: reset straight to stage 0
//   (auto)          claw rotator auto-preps to ROT_SCORE_DEG once a scoring
//                    stage is selected and the lift clears the frame; goes
//                    home otherwise
//   Right hold      claw rotator manual jog out (overrides auto, still
//                    blocked until lift clears the frame)
//   Y   hold        claw rotator manual jog home (overrides auto)
//   Up  hold        TEMPORARY: jog lift up, for measuring LIFT_STAGE_DEG
//   Down hold       TEMPORARY: jog lift down, for measuring LIFT_STAGE_DEG
//
void Mechanism::driverTick() {
    // --- L2: tap to step the stage up ---
    if (controller.pressed(Button::L2)) {
        stage_ = std::min(stage_ + 1, tune::LIFT_STAGE_COUNT - 1);
    }

    // --- R2: tap to step down, hold ~1s to reset to stage 0 instead ---
    if (controller.holding(Button::R2)) {
        if (r2HeldSince_ == 0) r2HeldSince_ = pros::millis();
        if (!r2ResetFired_ && pros::millis() - r2HeldSince_ >= tune::R2_RESET_HOLD_MS) {
            stage_ = 0;
            r2ResetFired_ = true;
        }
    } else {
        if (r2HeldSince_ != 0 && !r2ResetFired_) {
            stage_ = std::max(stage_ - 1, 0);
        }
        r2HeldSince_ = 0;
        r2ResetFired_ = false;
    }

    int jogDir = 0;
    if (controller.holding(Button::Up)) jogDir = 1;
    else if (controller.holding(Button::Down)) jogDir = -1;

    const int liftPos = static_cast<int>(lift.get_position());
    if (jogDir > 0 && liftPos >= tune::LIFT_MAX_DEG) jogDir = 0;
    if (jogDir < 0 && liftPos <= tune::LIFT_MIN_DEG) jogDir = 0;

    if (jogDir != 0) {
        lift.move(jogDir * tune::LIFT_JOG_CMD);
    } else {
        driveLift();
    }

    driveRotatorAndRollers();
    updateReadout();
}

void Mechanism::updateReadout() {
    const std::uint32_t now = pros::millis();
    if (now - lastReadoutAt_ < tune::READOUT_MS) return;
    lastReadoutAt_ = now;

    char buf[20];
    std::snprintf(buf, sizeof(buf), "S%d %d>%d", stage_,
                  tune::LIFT_STAGE_DEG[stage_], static_cast<int>(lift.get_position()));
    if (std::strncmp(buf, lastReadout_, sizeof(buf)) == 0) return;
    std::snprintf(lastReadout_, sizeof(lastReadout_), "%s", buf);
    controller.raw().set_text(2, 0, buf);
}

}
