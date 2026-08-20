#pragma once

#include <cstdint>
#include <functional>

#include "gen/pose.hpp"

namespace arc {

class ExitCondition {
    public:
        static bool error(float currentError, float threshold);
        static bool velocity(float currentVelocity, float threshold);
        static bool time(std::uint32_t startTime, int timeoutMs);
        static bool halfPlane(const Pose& pose, const Pose& target, float headingDeg, float tolerance);
};

// Sentinels preserve flat per-call syntax while omitted fields inherit the
// defaults declared beside the controller's PID gains.
inline constexpr int useProfileTimeout = -2147483647;
inline constexpr int useProfileSettleTime = -2147483646;
inline constexpr float useProfileExit = -3.402823466e+38f;
inline constexpr std::int8_t useProfileHalfPlane = -1;

struct ExitSettings {
    int timeout = 2000;
    float velocityExit = -1.0f;
    float errorExit = -1.0f;
    bool halfPlaneExit = false;
    float halfPlaneTolerance = 0.0f;
    int settleTimeMs = 0;
};

class Settler {
    public:
        explicit Settler(std::function<bool()> condition);

        Settler operator&(const Settler& other) const;
        Settler operator|(const Settler& other) const;
        bool isSettled() const;

    private:
        std::function<bool()> condition;
};

} // namespace arc
