#include "gen/features.hpp"

#if ARC_RAMSETE_LQR_ENABLED

#include "autons.hpp"
#include "gen/asset.hpp"
#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

extern arc::Chassis chassis;

ASSET(autonomous_seg0_path_txt);
ASSET(autonomous_seg1_path_txt);
ASSET(autonomous_seg2_path_txt);

void Auton::ramseteLqrTauTest() {
    constexpr float kStepDuty = 80.0f;
    constexpr int kDurationMs = 1500;
    constexpr int kSampleMs = 10;

    chassis.setPose(0, 0, 0);
    FILE* log = std::fopen("/usd/ramsete_tau_step.csv", "w");
    if (log != nullptr) std::fprintf(log, "t_ms,in_per_sec\n");

    const std::uint32_t start = pros::millis();
    std::uint32_t t = 0;
    while (t <= static_cast<std::uint32_t>(kDurationMs)) {
        chassis.tank(static_cast<int>(kStepDuty), static_cast<int>(kStepDuty));
        const arc::Pose velocity = arc::getSpeed(false);
        const float speed = std::hypot(velocity.x, velocity.y);
        if (log != nullptr) std::fprintf(log, "%lu,%.4f\n", static_cast<unsigned long>(t), speed);
        pros::delay(kSampleMs);
        t = pros::millis() - start;
    }
    chassis.tank(0, 0);
    if (log != nullptr) std::fclose(log);
}

void Auton::ramseteLqrTestRoutine() {
    chassis.setPose(-60.0, -0.0, 0.0);

    chassis.turnToPoint(-24.0, -24.0, {.timeout = 868});
    arc::followRamseteLQR(chassis, autonomous_seg0_path_txt, {
        .timeout = 2173,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
    pros::delay(100);
    arc::followRamseteLQR(chassis, autonomous_seg1_path_txt, {
        .timeout = 2874,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
    pros::delay(100);
    chassis.turnToPoint(48.0, -0.0, {.timeout = 591});
    arc::followRamseteLQR(chassis, autonomous_seg2_path_txt, {
        .timeout = 2192,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
}

#endif
