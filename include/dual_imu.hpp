#pragma once
// Two V5 Inertial Sensors that look like one pros::Imu to the chassis and odometry.
// The fusion rules are in imu_fusion.hpp. This file only talks to the hardware.
//
// Mounting (bottom C-channel, both on their side)
//  - Where on the robot does not matter for heading. Every point of a rigid robot
//    turns at the same rate. What matters is that the sensor is screwed down tight
//    to a part that does not flex or buzz, and sits square: exactly on its side,
//    not tilted.
//  - The sensor works out which way is up during calibration, so on its side is
//    fine. Keep the robot flat and still for the 2 to 3 s calibration at power-on.
//  - Mount the two opposite ways (back to back, one logo facing left, one facing
//    right). Each sensor drifts a little in a fixed direction relative to its own
//    body. Flipped, those drifts show up as opposite turns, so the average cancels
//    most of it.
//
// Scale, once per sensor after mounting (main.cpp kImuScaleA / kImuScaleB)
//  1. Set both scales to 1.0, power on with the robot flat and square against a wall.
//  2. Turn the robot by hand exactly 10 full turns clockwise, back square on the wall.
//  3. Read "IMU A raw" and "IMU B raw" on the brain screen. Scale = 3600 / raw.
//     If raw is negative that sensor counts backwards, the negative scale fixes it.
//  Turning the sensor onto its side uses a different gyro axis, so an old scale
//  measured flat does not carry over.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <mutex>

#include "imu_fusion.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace arc {

class DualIMU : public pros::Imu {
  public:
    // A sensor still calibrating after this long is left out of the run.
    static constexpr std::uint32_t kCalTimeoutMs = 4000;

    DualIMU(std::uint8_t portA, std::uint8_t portB, imu_fusion::Config config = {})
      : pros::Imu(portA), a_(portA), b_(portB), fuser_(config) {}

    std::int32_t reset(bool blocking = false) const override {
        {
            std::lock_guard<pros::Mutex> lock(mutex_);
            a_.reset(false);
            b_.reset(false);
            calStartMs_ = pros::millis();
            calibrating_ = true;
        }
        if (blocking) while (is_calibrating()) pros::delay(10);
        return 1;
    }

    bool is_calibrating() const override {
        std::lock_guard<pros::Mutex> lock(mutex_);
        if (!calibrating_) return false;
        const std::uint32_t elapsed = pros::millis() - calStartMs_;
        bool anyCalibrating = elapsed < 100;  // status takes a moment to show calibrating
        for (int i = 0; i < imu_fusion::kCount; ++i)
            if (alive(i) && sensor(i).is_calibrating()) anyCalibrating = true;
        if (anyCalibrating && elapsed < kCalTimeoutMs) return true;
        finishCalibrationLocked();
        return false;
    }

    pros::ImuStatus get_status() const override {
        std::lock_guard<pros::Mutex> lock(mutex_);
        if (calibrating_) return pros::ImuStatus::calibrating;
        return fuser_.activeCount() > 0 ? pros::ImuStatus::ready : pros::ImuStatus::error;
    }

    double get_rotation() const override {
        std::lock_guard<pros::Mutex> lock(mutex_);
        return updateLocked();
    }

    double get_heading() const override {
        std::lock_guard<pros::Mutex> lock(mutex_);
        const double r = updateLocked();
        if (!std::isfinite(r)) return r;
        const double h = std::fmod(r, 360.0);
        return h < 0.0 ? h + 360.0 : h;
    }

    double get_yaw() const override {
        const double h = get_heading();
        return std::isfinite(h) && h > 180.0 ? h - 360.0 : h;
    }

    // Roll and pitch come from one sensor (A if it is in use). Only crossBarrier
    // and the push motions read roll. Check its sign on the robot before using them,
    // a sensor on its side may report roll on a different axis.
    double get_roll() const override { return primary() ? primary()->get_roll() : PROS_ERR_F; }
    double get_pitch() const override { return primary() ? primary()->get_pitch() : PROS_ERR_F; }

    std::int32_t set_rotation(const double target) const override { return setLocked(target); }
    std::int32_t set_heading(const double target) const override { return setLocked(target); }
    std::int32_t tare_rotation() const override { return setLocked(0.0); }
    std::int32_t tare_heading() const override { return setLocked(0.0); }
    std::int32_t tare() const override { return setLocked(0.0); }

    // For the brain screen and scale measurement. Unscaled reading of one sensor.
    double rawRotation(int i) const {
        const double r = sensor(i).get_rotation();
        return std::isfinite(r) ? r : NAN;
    }
    bool inUse(int i) const {
        std::lock_guard<pros::Mutex> lock(mutex_);
        return fuser_.inUse(i);
    }
    int activeCount() const {
        std::lock_guard<pros::Mutex> lock(mutex_);
        return fuser_.activeCount();
    }

  private:
    const pros::Imu& sensor(int i) const { return i == 0 ? a_ : b_; }
    bool alive(int i) const { return sensor(i).get_status() != pros::ImuStatus::error; }
    const pros::Imu* primary() const {
        if (inUse(0)) return &a_;
        if (inUse(1)) return &b_;
        return nullptr;
    }

    imu_fusion::Sample sample(int i) const {
        const double r = sensor(i).get_rotation();
        return {std::isfinite(r), r};
    }

    // Reads only the sensors still in use, so a dropped or missing sensor cannot
    // leave an error code behind for odometry's errno check.
    double updateLocked() const {
        if (calibrating_) return PROS_ERR_F;
        imu_fusion::Sample s[imu_fusion::kCount];
        for (int i = 0; i < imu_fusion::kCount; ++i)
            if (fuser_.inUse(i)) s[i] = sample(i);
        const int before = fuser_.activeCount();
        const double r = fuser_.step(s);
        if (fuser_.activeCount() < before) report("dropped mid-run");
        if (!std::isfinite(r)) { errno = ENODEV; return PROS_ERR_F; }
        errno = 0;
        return r;
    }

    std::int32_t setLocked(double target) const {
        std::lock_guard<pros::Mutex> lock(mutex_);
        updateLocked();
        fuser_.setRotation(target);
        return 1;
    }

    void finishCalibrationLocked() const {
        calibrating_ = false;
        imu_fusion::Sample s[imu_fusion::kCount];
        for (int i = 0; i < imu_fusion::kCount; ++i) {
            const bool ok = alive(i) && !sensor(i).is_calibrating();
            s[i] = ok ? sample(i) : imu_fusion::Sample{};
        }
        fuser_.restart(s, 0.0);
        report("calibrated");
        errno = 0;
    }

    void report(const char* what) const {
        static const char* names[] = {"Z up", "Z down", "X up", "X down", "Y up", "Y down"};
        for (int i = 0; i < imu_fusion::kCount; ++i) {
            const int o = static_cast<int>(sensor(i).get_physical_orientation());
            std::printf("[imu %c] port %d, %s, %s, mounted %s\n", 'A' + i, sensor(i).get_port(), what,
                        fuser_.inUse(i) ? "in use" : "NOT USED", o >= 0 && o < 6 ? names[o] : "unknown");
        }
        if (fuser_.activeCount() < imu_fusion::kCount)
            pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "-");
    }

    pros::Imu a_, b_;
    mutable imu_fusion::Fuser fuser_;
    mutable pros::Mutex mutex_;
    mutable bool calibrating_ = false;
    mutable std::uint32_t calStartMs_ = 0;
};

}  // namespace arc
