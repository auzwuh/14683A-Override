#include "api.h"
namespace pros { inline namespace v5 {
pros::DeviceType Device::get_plugged_type() const { return {}; }
pros::DeviceType Device::get_plugged_type(std::uint8_t port)  { return {}; }
std::vector<Device> Device::get_all_devices(pros::DeviceType device_type)  { return {}; }
std::int32_t Motor::move(std::int32_t voltage) const { return {}; }
std::int32_t Motor::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_relative(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_velocity(const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_voltage(const std::int32_t voltage) const { return {}; }
std::int32_t Motor::brake(void) const { return {}; }
std::int32_t Motor::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double Motor::get_target_position(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_target_velocity(const std::uint8_t index) const { return {}; }
double Motor::get_actual_velocity(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_current_draw(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_direction(const std::uint8_t index) const { return {}; }
double Motor::get_efficiency(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_faults(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_flags(const std::uint8_t index) const { return {}; }
double Motor::get_position(const std::uint8_t index) const { return {}; }
double Motor::get_power(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
double Motor::get_temperature(const std::uint8_t index) const { return {}; }
double Motor::get_torque(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_voltage(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_over_current(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_over_temp(const std::uint8_t index) const { return {}; }
MotorBrake Motor::get_brake_mode(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_current_limit(const std::uint8_t index) const { return {}; }
MotorUnits Motor::get_encoder_units(const std::uint8_t index) const { return {}; }
MotorGears Motor::get_gearing(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_voltage_limit(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_reversed(const std::uint8_t index) const { return {}; }
MotorType Motor::get_type(const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_current_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const MotorUnits units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const MotorGears gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const pros::motor_gearset_e_t gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_reversed(const bool reverse, const std::uint8_t index)  { return {}; }
std::int32_t Motor::set_voltage_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_zero_position(const double position, const std::uint8_t index) const { return {}; }
std::int32_t Motor::tare_position(const std::uint8_t index) const { return {}; }
std::int8_t Motor::size(void) const { return {}; }
std::vector<Motor> Motor::get_all_devices()  { return {}; }
std::int8_t Motor::get_port(const std::uint8_t index) const { return {}; }
std::vector<double> Motor::get_target_position_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_target_velocity_all(void) const { return {}; }
std::vector<double> Motor::get_actual_velocity_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_current_draw_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_direction_all(void) const { return {}; }
std::vector<double> Motor::get_efficiency_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_faults_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_flags_all(void) const { return {}; }
std::vector<double> Motor::get_position_all(void) const { return {}; }
std::vector<double> Motor::get_power_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
std::vector<double> Motor::get_temperature_all(void) const { return {}; }
std::vector<double> Motor::get_torque_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_voltage_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_current_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_temp_all(void) const { return {}; }
std::vector<MotorBrake> Motor::get_brake_mode_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_current_limit_all(void) const { return {}; }
std::vector<MotorUnits> Motor::get_encoder_units_all(void) const { return {}; }
std::vector<MotorGears> Motor::get_gearing_all(void) const { return {}; }
std::vector<std::int8_t> Motor::get_port_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_voltage_limit_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_reversed_all(void) const { return {}; }
std::vector<MotorType> Motor::get_type_all(void) const { return {}; }
std::int32_t Motor::set_brake_mode_all(const MotorBrake mode) const { return {}; }
std::int32_t Motor::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const { return {}; }
std::int32_t Motor::set_current_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const MotorUnits units) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const { return {}; }
std::int32_t Motor::set_gearing_all(const MotorGears gearset) const { return {}; }
std::int32_t Motor::set_gearing_all(const pros::motor_gearset_e_t gearset) const { return {}; }
std::int32_t Motor::set_reversed_all(const bool reverse)  { return {}; }
std::int32_t Motor::set_voltage_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_zero_position_all(const double position) const { return {}; }
std::int32_t Motor::tare_position_all(void) const { return {}; }
std::int32_t MotorGroup::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double MotorGroup::get_target_position(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_target_position_all(void) const { return {}; }
std::int32_t MotorGroup::get_target_velocity(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_target_velocity_all(void) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_current_draw_all(void) const { return {}; }
std::int32_t MotorGroup::get_direction(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_direction_all(void) const { return {}; }
double MotorGroup::get_efficiency(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_efficiency_all(void) const { return {}; }
std::uint32_t MotorGroup::get_faults(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_faults_all(void) const { return {}; }
std::uint32_t MotorGroup::get_flags(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_flags_all(void) const { return {}; }
double MotorGroup::get_power(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_power_all(void) const { return {}; }
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
std::vector<double> MotorGroup::get_temperature_all(void) const { return {}; }
std::vector<double> MotorGroup::get_torque_all(void) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_current(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_current_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_temp(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_temp_all(void) const { return {}; }
std::int32_t MotorGroup::get_current_limit(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_current_limit_all(void) const { return {}; }
std::int32_t MotorGroup::get_voltage_limit(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_limit_all(void) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_reversed_all(void) const { return {}; }
MotorType MotorGroup::get_type(const std::uint8_t index) const { return {}; }
std::vector<MotorType> MotorGroup::get_type_all(void) const { return {}; }
std::int32_t MotorGroup::set_current_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_current_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<pros::motor_gearset_e_t> gearsets) const { return {}; }
std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t gearset, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<MotorGears> gearsets) const { return {}; }
std::int32_t MotorGroup::set_gearing(const MotorGears gearset, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const MotorGears gearset) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const pros::motor_gearset_e_t gearset) const { return {}; }
std::int32_t MotorGroup::set_voltage_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t limit) const { return {}; }
void MotorGroup::operator+=(AbstractMotor&)  {}
void MotorGroup::append(AbstractMotor&)  {}
void MotorGroup::erase_port(std::int8_t port)  {}
Imu Imu::get_imu()  { return Imu(std::uint8_t{0}); }
std::vector<Imu> Imu::get_all_devices()  { return {}; }
pros::quaternion_s_t Imu::get_quaternion() const { return {}; }
pros::euler_s_t Imu::get_euler() const { return {}; }
pros::imu_gyro_s_t Imu::get_gyro_rate() const { return {}; }
std::int32_t Imu::tare_pitch() const { return {}; }
std::int32_t Imu::tare_roll() const { return {}; }
std::int32_t Imu::tare_euler() const { return {}; }
std::int32_t Imu::set_pitch(const double target) const { return {}; }
std::int32_t Imu::set_roll(const double target) const { return {}; }
std::int32_t Imu::set_euler(const pros::euler_s_t target) const { return {}; }
pros::imu_accel_s_t Imu::get_accel() const { return {}; }
imu_orientation_e_t Imu::get_physical_orientation() const { return {}; }
std::vector<Rotation> Rotation::get_all_devices()  { return {}; }
std::vector<Distance> Distance::get_all_devices()  { return {}; }
}}
