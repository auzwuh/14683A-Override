#pragma once
// Driver control mixing. Left stick Y is forward and back, right stick X is turn.
// No PROS includes so tests/drive_mix_check.cpp can run it on a PC.

#include <algorithm>
#include <cmath>
#include <utility>

namespace drive_mix {

// Tuning knobs
inline constexpr double kDeadband = 0.04;     // fraction of stick ignored around centre (about 5 of 127)
inline constexpr double kThrottleExpo = 0.2;  // 0 = linear, 1 = cubic. Higher gives finer slow driving
inline constexpr double kTurnExpo = 0.4;      // higher gives finer small corrections, full stick is unchanged
inline constexpr double kTurnScale = 1.0;     // overall turn authority. Lower = wider arcs, slower point turns
// Who gives way when forward + turn ask for more than a side can do.
// 0 = both shrink together (full speed full turn stops the inner wheel, turns slowly)
// 1 = turn wins (full speed full turn spins in place, forward drops to 0)
// 0.6 = in between: full speed full turn reverses the inner wheel a bit, still rolls forward
inline constexpr double kTurnPriority = 0.6;
// Tuned on paper for 2.75 in traction wheels at 450 rpm, 10.63 in track (about 65 in/s,
// 700 deg/s full spin before wheel scrub, which makes real turns slower).
// Small turn input (10%) is about 27 deg/s, half stick about 236 deg/s standing still.
// Full forward + full turn is about 18 in/s forward while turning about 500 deg/s.

// Stick to -1..1. The deadband edge ramps from 0 instead of stepping, then expo
// softens the middle while keeping full stick at full output.
inline double shape(int raw, double expo) {
    const double x = std::clamp(raw / 127.0, -1.0, 1.0);
    const double mag = std::fabs(x);
    if (mag <= kDeadband) return 0.0;
    const double y = std::copysign((mag - kDeadband) / (1.0 - kDeadband), x);
    return (1.0 - expo) * y + expo * y * y * y;
}

// Arcade mix. When forward + turn would pass full power, forward first gives up
// kTurnPriority of the overflow, then both sides scale by the same factor so the
// arc is kept. Everything is continuous, so turn and forward both respond at once
// at any speed. Returns tank powers in -127..127.
inline std::pair<int, int> mix(int rawThrottle, int rawTurn) {
    const double s = shape(rawTurn, kTurnExpo) * kTurnScale;
    double t = shape(rawThrottle, kThrottleExpo);
    const double over = std::max(0.0, std::fabs(t) + std::fabs(s) - 1.0);
    t = std::copysign(std::max(0.0, std::fabs(t) - kTurnPriority * over), t);
    const double l = t + s;
    const double r = t - s;
    const double peak = std::max({1.0, std::fabs(l), std::fabs(r)});
    return {static_cast<int>(std::lround(127.0 * l / peak)),
            static_cast<int>(std::lround(127.0 * r / peak))};
}

// Lift soft stops for driver control
inline constexpr double kLiftSlowDeg = 15.0;   // lift degrees before an end where power starts to ramp down
inline constexpr double kLiftEdgeFrac = 0.25;  // power left right at the end of the ramp

// cmd -127..127 (+ = up), angleDeg from the bottom stop, topDeg the top of the
// stroke. Power toward an end ramps down over the last kLiftSlowDeg and is 0 past
// it. Power away from an end is never cut. An unknown angle or top skips that limit.
inline int liftPower(int cmd, double angleDeg, double topDeg) {
    if (cmd == 0 || !std::isfinite(angleDeg)) return cmd;
    const double room = cmd > 0 ? topDeg - angleDeg : angleDeg;
    if (!std::isfinite(room) || room >= kLiftSlowDeg) return cmd;
    if (room <= 0.0) return 0;
    return static_cast<int>(std::lround(cmd * (kLiftEdgeFrac + (1.0 - kLiftEdgeFrac) * room / kLiftSlowDeg)));
}

}  // namespace drive_mix
