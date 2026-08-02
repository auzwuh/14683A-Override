#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <initializer_list>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

namespace gen {

class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    double get_rotation() const override {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

class Piston : public pros::adi::DigitalOut {
 public:
  explicit Piston(std::uint8_t adi_port, bool init_state = false, std::string name = "")
      : pros::adi::DigitalOut(adi_port, init_state), state_(init_state), name_(std::move(name)) {}

  explicit Piston(pros::adi::ext_adi_port_pair_t port_pair, bool init_state = false, std::string name = "")
      : pros::adi::DigitalOut(port_pair, init_state), state_(init_state), name_(std::move(name)) {}

  std::int32_t set_value(bool value) {
    const auto result = pros::adi::DigitalOut::set_value(value);
    if (result != PROS_ERR) {
      state_ = value;
    }
    return result;
  }

  void on() { set_value(true); }
  void off() { set_value(false); }
  void toggle() { set_value(!state_); }

  bool state() const { return state_; }
  const std::string& name() const { return name_; }
  using pros::adi::DigitalOut::get_port;

 private:
  bool state_{false};
  std::string name_{};
};

struct PistonHandle {
  std::string name;
  Piston* piston{nullptr};
  bool enabled{true};
};

class PistonGroup {
 public:
  PistonGroup() = default;
  PistonGroup(std::initializer_list<PistonHandle> pistons) {
    for (const auto& piston : pistons) add(piston);
  }

  void add(const PistonHandle& piston) {
    if (piston.piston == nullptr) return;
    const std::string key = piston.name.empty() ? std::to_string(pistons_.size()) : piston.name;
    pistons_[key] = piston;
  }

  void remove(const std::string& name) { pistons_.erase(name); }
  void enable(const std::string& name) { pistons_.at(name).enabled = true; }
  void disable(const std::string& name) { pistons_.at(name).enabled = false; }

  void on() {
    for (auto& [_, piston] : pistons_) {
      if (piston.enabled && piston.piston != nullptr) piston.piston->on();
    }
  }

  void off() {
    for (auto& [_, piston] : pistons_) {
      if (piston.enabled && piston.piston != nullptr) piston.piston->off();
    }
  }

  void toggle() {
    const bool targetState = !state();
    targetState ? on() : off();
  }

  bool state() const {
    for (const auto& [_, piston] : pistons_) {
      if (piston.enabled && piston.piston != nullptr) return piston.piston->state();
    }
    return false;
  }

 private:
  std::map<std::string, PistonHandle> pistons_;
};

class MotorGroup : public pros::MotorGroup {
 public:
  MotorGroup(std::initializer_list<std::int8_t> ports, double motor_rpm, double gear_ratio)
      : pros::MotorGroup(ports, gearset_from_rpm(motor_rpm)),
        motor_rpm_(motor_rpm),
        gear_ratio_(gear_ratio),
        motors_(build_motors(ports, gearset_from_rpm(motor_rpm))) {}

  MotorGroup(const std::vector<std::int8_t>& ports, double motor_rpm, double gear_ratio)
      : pros::MotorGroup(ports, gearset_from_rpm(motor_rpm)),
        motor_rpm_(motor_rpm),
        gear_ratio_(gear_ratio),
        motors_(build_motors(ports, gearset_from_rpm(motor_rpm))) {}

  pros::Motor& operator[](std::size_t index) { return motors_.at(index); }
  const pros::Motor& operator[](std::size_t index) const { return motors_.at(index); }

  // Move with a normalized percent [-1, 1], keeping the original move() available.
  void move_percent(double percent) {
    const double clamped = clamp_percent(percent);
    pros::MotorGroup::move(static_cast<std::int32_t>(clamped * 127.0));
  }

  // Move the motors in motor RPM.
  void move_rpm(double rpm) { pros::MotorGroup::move_velocity(rpm); }

  // Move based on wheel RPM; converts to motor RPM using the stored gear ratio.
  void move_wheel_rpm(double wheel_rpm) { pros::MotorGroup::move_velocity(wheel_rpm * gear_ratio_); }

  double motor_rpm() const { return motor_rpm_; }
  double gear_ratio() const { return gear_ratio_; }
  double wheel_rpm() const { return gear_ratio_ == 0 ? 0 : motor_rpm_ / gear_ratio_; }

 private:
  static std::vector<pros::Motor> build_motors(std::initializer_list<std::int8_t> ports,
                                               pros::v5::MotorGears gearset) {
    std::vector<pros::Motor> motors;
    motors.reserve(ports.size());
    for (auto port : ports) {
      motors.emplace_back(port, gearset);
    }
    return motors;
  }

  static std::vector<pros::Motor> build_motors(const std::vector<std::int8_t>& ports,
                                               pros::v5::MotorGears gearset) {
    std::vector<pros::Motor> motors;
    motors.reserve(ports.size());
    for (auto port : ports) {
      motors.emplace_back(port, gearset);
    }
    return motors;
  }

  static double clamp_percent(double percent) {
    if (percent > 1.0) {
      return 1.0;
    }
    if (percent < -1.0) {
      return -1.0;
    }
    return percent;
  }

  static pros::v5::MotorGears gearset_from_rpm(double rpm) {
    const double rounded = std::round(rpm);
    if (std::abs(rounded - 100.0) < 1.0) {
      return pros::v5::MotorGears::red;
    }
    if (std::abs(rounded - 200.0) < 1.0) {
      return pros::v5::MotorGears::green;
    }
    if (std::abs(rounded - 600.0) < 1.0) {
      return pros::v5::MotorGears::blue;
    }
    return pros::v5::MotorGears::invalid;
  }

  double motor_rpm_{0};
  double gear_ratio_{1};
  std::vector<pros::Motor> motors_{};
};

class Controller {
 public:
  enum class Button {
    R1,
    L1,
    R2,
    L2,
    A,
    B,
    X,
    Y,
    Up,
    Down,
    Left,
    Right
  };

  enum class DriveMode { Tank2Stick, Arcade1Stick, Arcade2Stick };

  Controller(DriveMode mode,
             int deadband = 0,
             double curve_amount = 10.0,
             bool drive_curves = true,
             pros::controller_id_e_t id = pros::E_CONTROLLER_MASTER)
      : controller_(id),
        mode_(mode),
        deadband_(deadband),
        curve_amount_(curve_amount),
        drive_curves_(drive_curves) {}

  std::pair<int, int> drive_values() const {
    switch (mode_) {
      case DriveMode::Tank2Stick:
        return tank_two_stick();
      case DriveMode::Arcade1Stick:
        return arcade_one_stick();
      case DriveMode::Arcade2Stick:
      default:
        return arcade_two_stick();
    }
  }

  pros::Controller& raw() { return controller_; }
  const pros::Controller& raw() const { return controller_; }

  void mode(DriveMode mode) { mode_ = mode; }
  void deadband(int deadband) { deadband_ = deadband; }
  void curve(double amount, bool enabled) {
    curve_amount_ = amount;
    drive_curves_ = enabled;
  }

  bool holding(Button button) const { return controller_.get_digital(to_pros(button)); }
  bool pressed(Button button) const { return controller_.get_digital_new_press(to_pros(button)); }

  std::pair<int, int> tank_two_stick() const {
    const int left = read_axis(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int right = read_axis(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    return {left, right};
  }

  std::pair<int, int> arcade_one_stick() const {
    const int forward = read_axis(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = read_axis(pros::E_CONTROLLER_ANALOG_LEFT_X);
    return mix_arcade(forward, turn);
  }

  std::pair<int, int> arcade_two_stick() const {
    const int forward = read_axis(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = read_axis(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    return mix_arcade(forward, turn);
  }

 private:
  static int apply_deadband(int value, int deadband) {
    return std::abs(value) < deadband ? 0 : value;
  }

  static int clamp_analog(int value) {
    if (value > 127) return 127;
    if (value < -127) return -127;
    return value;
  }

  int curve(int input) const {
    if (!drive_curves_) return input;
    const double base = std::exp(-curve_amount_ / 10.0);
    const double expo = std::exp((std::abs(input) - 127.0) / 10.0);
    const double scaled = (base + expo * (1.0 - base)) * static_cast<double>(input);
    return clamp_analog(static_cast<int>(std::round(scaled)));
  }

  int read_axis(pros::controller_analog_e_t channel) const {
    const int raw_val = controller_.get_analog(channel);
    return curve(apply_deadband(raw_val, deadband_));
  }

  static std::pair<int, int> mix_arcade(int forward, int turn) {
    const int left = clamp_analog(forward + turn);
    const int right = clamp_analog(forward - turn);
    return {left, right};
  }

  static pros::controller_digital_e_t to_pros(Button button) {
    switch (button) {
      case Button::R1: return pros::E_CONTROLLER_DIGITAL_R1;
      case Button::L1: return pros::E_CONTROLLER_DIGITAL_L1;
      case Button::R2: return pros::E_CONTROLLER_DIGITAL_R2;
      case Button::L2: return pros::E_CONTROLLER_DIGITAL_L2;
      case Button::A: return pros::E_CONTROLLER_DIGITAL_A;
      case Button::B: return pros::E_CONTROLLER_DIGITAL_B;
      case Button::X: return pros::E_CONTROLLER_DIGITAL_X;
      case Button::Y: return pros::E_CONTROLLER_DIGITAL_Y;
      case Button::Up: return pros::E_CONTROLLER_DIGITAL_UP;
      case Button::Down: return pros::E_CONTROLLER_DIGITAL_DOWN;
      case Button::Left: return pros::E_CONTROLLER_DIGITAL_LEFT;
      case Button::Right: return pros::E_CONTROLLER_DIGITAL_RIGHT;
    }
    return pros::E_CONTROLLER_DIGITAL_A;
  }

  mutable pros::Controller controller_;
  DriveMode mode_;
  int deadband_;
  double curve_amount_;
  bool drive_curves_;
};
}  // namespace gen
