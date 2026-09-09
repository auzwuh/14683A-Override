#pragma once

#include <cstdint>
#include "gen/chassis/chassis.hpp"
#include "gen/pose.hpp"

namespace arc {
/** Coherent inches/clockwise-compass-radians state. Signed body velocities are
 * inches/sec (forward, left positive), angular velocity clockwise rad/sec.
 * Valid requires unpowered pods on both axes, IMU and a measured interval.
 * First samples, resets and sensor faults publish valid=false.
 */
struct OdomSnapshot {
    Pose pose{0, 0, 0};
    float forwardVelocity = 0;
    float leftVelocity = 0;
    float clockwiseAngularVelocity = 0;
    std::uint32_t timestampMs = 0;
    float dtSeconds = 0;
    bool valid = false;
};
OdomSnapshot getOdomSnapshot();
/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(arc::OdomSensors sensors, arc::Drivetrain drivetrain);
/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);
/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by default
 */
void setPose(Pose pose, bool radians = false);
/**
 * @brief Get the speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return arc::Pose
 */
Pose getSpeed(bool radians = false);
/**
 * @brief Get the local speed of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return arc::Pose
 */
Pose getLocalSpeed(bool radians = false);
/**
 * @brief Estimate the pose of the robot after a certain amount of time
 *
 * @param time time in seconds
 * @param radians False for degrees, true for radians. False by default
 * @return arc::Pose
 */
Pose estimatePose(float time, bool radians = false);
/**
 * @brief Update the pose of the robot
 *
 */
void update();
/**
 * @brief Initialize the odometry system
 *
 */
void init();
} // namespace arc
