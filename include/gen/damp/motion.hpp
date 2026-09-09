#pragma once
#include "gen/damp/follower.hpp"
#include <cstdint>
#include <cstdio>
namespace arc::damp {
enum class MotionStatus { succeeded, disabled, invalidInput, sensorFault, lowBattery, motorFault, cancelled, timedOut };
struct MotionParams {
    FollowerConfig follower{};
    std::uint32_t timeoutMs=15000, maxSensorAgeMs=50;
};
struct MotionResult { MotionStatus status=MotionStatus::invalidInput; double referenceTime=0; };
struct CalibrationStep { std::uint32_t durationMs=0; Voltage voltage{}; };
inline Waypoint waypointInches(double x,double y) { return {x*.0254,y*.0254}; }
// Existing odometry uses inches and clockwise heading from field +Y.
inline State compassState(double x,double y,double heading,double forward,double left,double clockwiseYaw) {
    return {x*.0254,y*.0254,1.5707963267948966-heading,forward*.0254,left*.0254,-clockwiseYaw};
}
}
