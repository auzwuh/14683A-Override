#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "gen/path_following/all.hpp"

using namespace arc::path;

namespace {

constexpr double kDt = 0.05;
constexpr std::size_t kPreview = 18;

struct Metrics {
    double positionRmse = 0.0;
    double headingRmseDeg = 0.0;
    double finalPosition = 0.0;
    double maxPosition = 0.0;
    double commandJerk = 0.0;
    double microsecondsPerStep = 0.0;
};

Horizon makeReference() {
    Horizon result;
    Pose pose{};
    for (std::size_t i = 0; i <= 320; ++i) {
        const double time = i * kDt;
        const double ramp = std::min(1.0, time / 1.0);
        const double stopRamp = std::clamp((14.0 - time) / 1.0, 0.0, 1.0);
        const double velocity = 38.0 * ramp * stopRamp;
        // Alternating curvature creates an S, then a tighter late turn.
        const double omega = velocity == 0.0 ? 0.0
            : (0.42 * std::sin(0.72 * time) + (time > 8.0 ? 0.16 : 0.0));
        result.push_back({pose, {velocity, omega}});
        pose = stepUnicycle(pose, {velocity, omega}, kDt);
    }
    return result;
}

Horizon preview(const Horizon& full, std::size_t start) {
    Horizon result;
    result.reserve(kPreview + 1);
    for (std::size_t i = 0; i <= kPreview; ++i)
        result.push_back(full[std::min(start + i, full.size() - 1)]);
    return result;
}

template <typename Controller>
Metrics run(Controller& controller, const Horizon& reference) {
    State state{{reference.front().pose.x - 5.0, reference.front().pose.y + 7.0,
                 reference.front().pose.theta + 18.0 * std::numbers::pi / 180.0}, 0.0, 0.0};
    Control previous{};
    double positionSquares = 0.0;
    double headingSquares = 0.0;
    double jerk = 0.0;
    double elapsedUs = 0.0;
    Metrics metrics;

    for (std::size_t i = 0; i + 1 < reference.size(); ++i) {
        if (i == 105) state.pose.y += 5.0; // identical lateral shove for every controller
        const Horizon local = preview(reference, i);
        const auto begin = std::chrono::steady_clock::now();
        const Control command = controller.calculate(state, local, kDt);
        const auto end = std::chrono::steady_clock::now();
        elapsedUs += std::chrono::duration<double, std::micro>(end - begin).count();

        // Common first-order drivetrain and a small, deterministic 4% speed loss.
        constexpr double tau = 0.11;
        const double alpha = 1.0 - std::exp(-kDt / tau);
        state.linearVelocity += alpha * (0.96 * command.linear - state.linearVelocity);
        state.angularVelocity += alpha * (command.angular - state.angularVelocity);
        state.pose = stepUnicycle(state.pose, {state.linearVelocity, state.angularVelocity}, kDt);

        const auto error = poseError(state.pose, reference[i + 1].pose);
        const double position = std::hypot(error[0], error[1]);
        positionSquares += position * position;
        headingSquares += error[2] * error[2];
        metrics.maxPosition = std::max(metrics.maxPosition, position);
        jerk += std::hypot(command.linear - previous.linear, command.angular - previous.angular);
        previous = command;
    }
    const double count = static_cast<double>(reference.size() - 1);
    metrics.positionRmse = std::sqrt(positionSquares / count);
    metrics.headingRmseDeg = std::sqrt(headingSquares / count) * 180.0 / std::numbers::pi;
    metrics.finalPosition = std::hypot(state.pose.x - reference.back().pose.x,
                                       state.pose.y - reference.back().pose.y);
    metrics.commandJerk = jerk / count;
    metrics.microsecondsPerStep = elapsedUs / count;
    return metrics;
}

void print(const std::string& name, const Metrics& metrics) {
    std::cout << std::left << std::setw(14) << name << std::right
              << std::setw(12) << metrics.positionRmse
              << std::setw(14) << metrics.headingRmseDeg
              << std::setw(12) << metrics.finalPosition
              << std::setw(12) << metrics.maxPosition
              << std::setw(12) << metrics.commandJerk
              << std::setw(14) << metrics.microsecondsPerStep << '\n';
}

}

int main() {
    const Horizon reference = makeReference();
    SamplingMPCController smpc;
    ILQRController ilqr;
    RTIController rti;
    RamseteController ramsete;
    LTVLQRController ltvlqr;
    RamseteLQRController ramseteLqr;

    std::cout << std::fixed << std::setprecision(3)
              << std::left << std::setw(14) << "controller" << std::right
              << std::setw(12) << "pos_rmse"
              << std::setw(14) << "head_rmse_deg"
              << std::setw(12) << "final_pos"
              << std::setw(12) << "max_pos"
              << std::setw(12) << "mean_du"
              << std::setw(14) << "us_per_step" << '\n';
    print("sampling_mpc", run(smpc, reference));
    print("ilqr", run(ilqr, reference));
    print("rti", run(rti, reference));
    print("ramsete", run(ramsete, reference));
    print("ltvlqr", run(ltvlqr, reference));
    print("ramsete_lqr", run(ramseteLqr, reference));
}
