#include "macros.hpp"

#include "gen/electronics.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern arc::Controller controller;
extern arc::MotorGroup intake;
extern arc::MotorGroup lift;
extern arc::MotorGroup claw;
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

bool groupJammed(arc::MotorGroup& group) {
    std::int32_t worstCurrent = 0;
    for (const std::int32_t mA : group.get_current_draw_all()) {
        if (mA > worstCurrent) worstCurrent = mA;
    }
    double slowest = 1e9;
    for (const double rpm : group.get_actual_velocity_all()) {
        const double mag = std::fabs(rpm);
        if (mag < slowest) slowest = mag;
    }
    return worstCurrent > tune::STALL_MA && slowest < tune::STALL_VEL_RPM;
}

}


void Mechanism::init() {
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    clawRot.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);


    lift.tare_position_all();
    clawRot.tare_position_all();
    claw.tare_position_all();

    stage_ = Stage::Manual;
    base_ = Base::Short;
    level_ = 0;
    liftCmd_ = tune::LIFT_STOW;
    rotCmd_ = tune::ROT_STOW;
    clawCmd_ = tune::CLAW_GRIP;
    snapValid_ = false;
    liftJogDir_ = 0;
    rotJogDir_ = 0;
    for (auto& row : trims_) {
        for (int& t : row) t = 0;
    }
}

// ------------------------------------------------------------- commands ----

void Mechanism::commandLift(int target, int velocity) {
    if (target == liftCmd_ && velocity == liftVel_) return;
    liftCmd_ = target;
    liftVel_ = velocity;
    lift.move_absolute(target, velocity);
}

void Mechanism::commandRot(int target) {
    if (target == rotCmd_) return;
    rotCmd_ = target;
    clawRot.move_absolute(target, tune::ROT_VEL);
}

void Mechanism::commandClaw(int target) {
    if (target == clawCmd_) return;
    clawCmd_ = target;
    claw.move_absolute(target, tune::CLAW_VEL);
}

bool Mechanism::liftAt(int target) const {
    return std::abs(static_cast<int>(lift.get_position()) - target) <= tune::LIFT_TOL;
}

bool Mechanism::rotAt(int target) const {
    return std::abs(static_cast<int>(clawRot.get_position()) - target) <= tune::ROT_TOL;
}

bool Mechanism::rotStowed() const {
    return rotAt(tune::ROT_STOW);
}

int Mechanism::liftFloor() const {
    // With the claw swung out, the lift may not come below the interlock.
    return rotStowed() ? tune::LIFT_MIN_DEG : tune::LIFT_ROT_CLEAR;
}


int Mechanism::levelDegrees(Base b, int level) const {
    const int anchor = (b == Base::Tall) ? tune::LIFT_TALL : tune::LIFT_SHORT;
    const int idx = (b == Base::Tall) ? 1 : 0;
    const int lvl = clampInt(level, 0, tune::MAX_LEVEL);
    const int raw = tune::levelSeedDeg(anchor, lvl) + trims_[idx][lvl];
    return clampInt(raw, tune::LIFT_MIN_DEG, tune::LIFT_MAX_DEG);
}


void Mechanism::enterStage(Stage s) {
    stage_ = s;
    stageAt_ = pros::millis();
    stallSince_ = 0;
    if (s != Stage::Manual) {
        liftJogDir_ = 0;
        rotJogDir_ = 0;
    }
}

void Mechanism::intakePose() {
    if (stage_ == Stage::Faulted) return;
    snapValid_ = false;
    commandClaw(tune::CLAW_OPEN);
    intake.move(127);
    enterStage(Stage::ToIntake);
}

void Mechanism::gotoLevel(int level) {
    if (stage_ == Stage::Faulted) return;
    level_ = clampInt(level, 0, tune::MAX_LEVEL);
    commandClaw(tune::CLAW_GRIP);
    intake.move(0);
    snapTarget_ = levelDegrees(base_, level_);
    snapRotStow_ = false;
    snapValid_ = true;
    enterStage(Stage::Snapping);
}

void Mechanism::nextLevel() { gotoLevel(level_ + 1); }
void Mechanism::prevLevel() { gotoLevel(level_ - 1); }

void Mechanism::scoreCup() {
    if (stage_ == Stage::Faulted) return;
    snapValid_ = false;
    commandClaw(tune::CLAW_OPEN);
    enterStage(Stage::Releasing);
}

void Mechanism::flipToggle() {
    if (stage_ == Stage::Faulted) return;
    commandClaw(tune::CLAW_GRIP);
    intake.move(0);
    snapTarget_ = clampInt(tune::LIFT_TOGGLE, tune::LIFT_MIN_DEG, tune::LIFT_MAX_DEG);
    snapRotStow_ = true;
    snapValid_ = false;
    enterStage(Stage::Snapping);
}

void Mechanism::setBase(Base b) {
    base_ = b;
    if (stage_ == Stage::Snapping && !snapRotStow_) gotoLevel(level_);
}

void Mechanism::abort() {
    stage_ = Stage::Manual;
    stallSince_ = 0;
    snapValid_ = false;
    liftJogDir_ = 0;
    rotJogDir_ = 0;
    captureLiftHold();
    captureRotHold();
    intake.move(0);
}


void Mechanism::captureLiftHold() {
    liftCmd_ = static_cast<int>(lift.get_position());
    liftVel_ = tune::LIFT_DOWN_VEL;
    lift.move_absolute(liftCmd_, liftVel_);
}

void Mechanism::captureRotHold() {
    rotCmd_ = static_cast<int>(clawRot.get_position());
    clawRot.move_absolute(rotCmd_, tune::ROT_VEL);
}

void Mechanism::jogLift(int dir) {
    const int pos = static_cast<int>(lift.get_position());
    if (dir > 0 && pos >= tune::LIFT_MAX_DEG) dir = 0;
    if (dir < 0 && pos <= liftFloor()) dir = 0;

    if (dir == liftJogDir_) {
        if (dir != 0) lift.move(dir > 0 ? tune::LIFT_JOG_UP_CMD : -tune::LIFT_JOG_DOWN_CMD);
        return;
    }
    liftJogDir_ = dir;
    if (dir == 0) {
        if (snapValid_) {
            const int idx = (base_ == Base::Tall) ? 1 : 0;
            const int lvl = clampInt(level_, 0, tune::MAX_LEVEL);
            trims_[idx][lvl] = clampInt(
                trims_[idx][lvl] + (static_cast<int>(lift.get_position()) - snapTarget_),
                -tune::TRIM_LIMIT, tune::TRIM_LIMIT);
            snapTarget_ = levelDegrees(base_, level_);
        }
        captureLiftHold();
    } else {
        lift.move(dir > 0 ? tune::LIFT_JOG_UP_CMD : -tune::LIFT_JOG_DOWN_CMD);
    }
}

void Mechanism::jogRot(int dir) {
    const int pos = static_cast<int>(clawRot.get_position());
    if (dir > 0 && (pos >= tune::ROT_MAX || lift.get_position() < tune::LIFT_ROT_CLEAR)) dir = 0;
    if (dir < 0 && pos <= tune::ROT_MIN) dir = 0;

    if (dir == rotJogDir_) {
        if (dir != 0) clawRot.move(dir > 0 ? tune::ROT_JOG_CMD : -tune::ROT_JOG_CMD);
        return;
    }
    rotJogDir_ = dir;
    if (dir == 0) {
        captureRotHold();
    } else {
        clawRot.move(dir > 0 ? tune::ROT_JOG_CMD : -tune::ROT_JOG_CMD);
    }
}


void Mechanism::checkStall() {
    if (stage_ == Stage::Faulted) {
        stallSince_ = 0;
        return;
    }
    const bool liftShouldMove =
        liftJogDir_ != 0 ||
        std::abs(static_cast<int>(lift.get_position()) - liftCmd_) > tune::LIFT_TOL;
    const bool rotShouldMove =
        rotJogDir_ != 0 ||
        std::abs(static_cast<int>(clawRot.get_position()) - rotCmd_) > tune::ROT_TOL;

    const bool jammed = (liftShouldMove && groupJammed(lift)) ||
                        (rotShouldMove && groupJammed(clawRot));
    if (!jammed) {
        stallSince_ = 0;
        return;
    }
    const std::uint32_t now = pros::millis();
    if (stallSince_ == 0) {
        stallSince_ = now;
        return;
    }
    if (now - stallSince_ >= static_cast<std::uint32_t>(tune::STALL_MS)) fault();
}

void Mechanism::fault() {
    liftJogDir_ = 0;
    rotJogDir_ = 0;
    captureLiftHold();
    captureRotHold();
    stage_ = Stage::Faulted;
    stallSince_ = 0;
    snapValid_ = false;
    controller.raw().rumble("---");
}

void Mechanism::tick() {
    checkStall();

    const std::uint32_t now = pros::millis();

    switch (stage_) {
        case Stage::Manual:
        case Stage::Faulted:
            break;

        case Stage::Snapping: {
            const int pos = static_cast<int>(lift.get_position());
            const int vel = (pos > snapTarget_ + tune::LIFT_TOL) ? tune::LIFT_DOWN_VEL
                                                                 : tune::LIFT_UP_VEL;
            if (snapRotStow_) {
                commandRot(tune::ROT_STOW);
                if (rotStowed() || snapTarget_ >= tune::LIFT_ROT_CLEAR) {
                    commandLift(snapTarget_, vel);
                }
                if (liftAt(snapTarget_) && rotStowed()) enterStage(Stage::Manual);
            } else {
                commandLift(snapTarget_, vel);
                if (pos >= tune::LIFT_ROT_CLEAR) commandRot(tune::ROT_SCORE);
                if (liftAt(snapTarget_) && rotAt(tune::ROT_SCORE)) enterStage(Stage::Manual);
            }
            break;
        }

        case Stage::Releasing:
            if (now - stageAt_ >= static_cast<std::uint32_t>(tune::RELEASE_DWELL_MS)) {
                level_ = clampInt(level_ + 1, 0, tune::MAX_LEVEL);
                intakePose();
            }
            break;

        case Stage::ToIntake: {
            commandRot(tune::ROT_STOW);
            if (rotStowed()) {
                commandLift(tune::LIFT_STOW, tune::LIFT_DOWN_VEL);
                if (liftAt(tune::LIFT_STOW)) enterStage(Stage::Manual);
            } else if (lift.get_position() < tune::LIFT_ROT_CLEAR) {
                // Too low for the rotator to swing home - go up to the clearance
                // height first, then come back down.
                commandLift(tune::LIFT_ROT_CLEAR, tune::LIFT_UP_VEL);
            }
            break;
        }
    }
}


void Mechanism::updateReadout() {
    const std::uint32_t now = pros::millis();
    if (now - lastReadoutAt_ < static_cast<std::uint32_t>(tune::READOUT_MS)) return;
    lastReadoutAt_ = now;

    char buf[20];
    if (stage_ == Stage::Faulted) {
        std::snprintf(buf, sizeof(buf), "JAM - Left");
    } else {
        std::snprintf(buf, sizeof(buf), "%c%d %d>%d",
                      base_ == Base::Tall ? 'T' : 'S',
                      level_,
                      levelDegrees(base_, level_),
                      static_cast<int>(lift.get_position()));
    }
    if (std::strncmp(buf, lastReadout_, sizeof(buf)) == 0) return;
    std::snprintf(lastReadout_, sizeof(lastReadout_), "%s", buf);
    controller.raw().set_text(2, 0, buf);
}

//
//   L1  hold    intake in
//   R1  hold    outtake
//   L2  hold    cascade up
//   R2  hold    cascade down
//   Right hold  claw rotator up
//   Y   hold    claw rotator down
//   A           score Cup: release, settle, reset down, level++
//   Up          next stack level
//   Down        previous stack level
//   Left        reset to intake pose / clear a stall fault
//   X           claw grip <-> open
//   B           toggle Short <-> Tall Goal ladder
//
void Mechanism::driverTick() {
    if (controller.holding(Button::L1)) {
        intake.move(127);
    } else if (controller.holding(Button::R1)) {
        intake.move(-127);
    } else if (stage_ != Stage::ToIntake && stage_ != Stage::Releasing) {
        intake.move(0);
    }
    int liftDir = 0;
    if (controller.holding(Button::L2)) liftDir = 1;
    else if (controller.holding(Button::R2)) liftDir = -1;

    int rotDir = 0;
    if (controller.holding(Button::Right)) rotDir = 1;
    else if (controller.holding(Button::Y)) rotDir = -1;

    if ((liftDir != 0 || rotDir != 0) && stage_ != Stage::Faulted &&
        stage_ != Stage::Releasing) {
        if (stage_ != Stage::Manual) enterStage(Stage::Manual);
    }
    if (stage_ == Stage::Manual) {
        jogLift(liftDir);
        jogRot(rotDir);
    }

    // --- A: score the Cup ---
    if (controller.pressed(Button::A) && stage_ != Stage::Faulted) scoreCup();

    // --- stack level ---
    if (controller.pressed(Button::Up)) nextLevel();
    if (controller.pressed(Button::Down)) prevLevel();

    // --- Left: reset, and the way out of a stall fault ---
    if (controller.pressed(Button::Left)) {
        if (faulted()) {
            abort();
        } else {
            level_ = 0;
            intakePose();
        }
    }

    // --- manual claw ---
    if (controller.pressed(Button::X)) {
        commandClaw(clawCmd_ == tune::CLAW_GRIP ? tune::CLAW_OPEN : tune::CLAW_GRIP);
    }

    // --- which Goal the ladder is anchored on ---
    if (controller.pressed(Button::B)) {
        setBase(base_ == Base::Short ? Base::Tall : Base::Short);
    }

    tick();
    updateReadout();
}

}
