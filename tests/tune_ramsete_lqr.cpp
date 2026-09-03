// Random search over every RamseteLQRConfig knob (b, zeta,
// velocityTimeConstant, and the four LQR tolerances), scored against the
// same S-curve scenario and the same three stress conditions as
// path_following_benchmark.cpp / docs/path_following_notebook.md
// (Experiments 1-3: baseline, heavier lag/slip/disturbance, short preview).
// Reusing that exact plant model - not a new one - is what makes the result
// trustworthy: it is the model the whole controller comparison already
// leaned on to pick RamseteLQRController in the first place.
//
// This is a desktop simulation search, not a substitute for measuring
// velocityTimeConstant on the real robot (see Auton::ramseteLqrTauTest in
// src/main.cpp) - it tells you which config is best *for a given plant*,
// including whatever tau you plug in as the assumed truth per scenario.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

#include "gen/path_following/all.hpp"

using namespace arc::path;

namespace {

constexpr double kDt = 0.05;

// Deterministic xorshift32 - same generator/rationale as
// SamplingMPCController::uniformSigned: repeatable sweeps, no RNG library.
class Xorshift32 {
  public:
    explicit Xorshift32(std::uint32_t seed) : state_(seed ? seed : 1u) {}
    double uniform01() {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return static_cast<double>(state_) / 4294967295.0;
    }
    double logUniform(double low, double high) {
        return low * std::pow(high / low, uniform01());
    }
    double uniform(double low, double high) { return low + (high - low) * uniform01(); }
    // Box-Muller: turns two uniforms into one N(0,1) draw.
    double gaussian() {
        const double u1 = std::max(uniform01(), 1e-12); // avoid log(0)
        const double u2 = uniform01();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
    }

  private:
    std::uint32_t state_;
};

// One of the notebook's three stress conditions, plus a fourth added here:
// sensor noise. The other three only stress the *plant*; none of them give
// an unboundedly aggressive correction gain any reason to lose - a noiseless
// simulation can only ever reward more stiffness. Real odometry is not
// noiseless, so without this scenario the search has no way to discover the
// real accuracy-vs-chatter tradeoff and just walks b to whatever ceiling
// it's given. Noise magnitudes below are an illustrative placeholder
// (typical tracking-wheel/IMU jitter order of magnitude) - replace with
// real measured V5 sensor noise if/when that's characterized.
struct Scenario {
    const char* name;
    double lagTau;          // drivetrain first-order velocity time constant
    double speedLoss;        // fractional speed loss applied to every command
    double shoveInches;      // one-time lateral disturbance
    std::size_t previewLen;  // horizon steps handed to calculate()
    double positionNoiseIn;  // stddev of position measurement noise fed to the controller
    double headingNoiseRad;  // stddev of heading measurement noise fed to the controller
};

constexpr Scenario kScenarios[] = {
    {"baseline", 0.11, 0.04, 5.0, 18, 0.0, 0.0},
    {"heavier_lag_slip_dist", 0.25, 0.10, 10.0, 18, 0.0, 0.0},
    {"short_preview", 0.11, 0.04, 5.0, 6, 0.0, 0.0},
    {"sensor_noise", 0.11, 0.04, 5.0, 18, 0.08, 0.3 * std::numbers::pi / 180.0},
};

Horizon makeReference() {
    Horizon result;
    Pose pose{};
    for (std::size_t i = 0; i <= 320; ++i) {
        const double time = i * kDt;
        const double ramp = std::min(1.0, time / 1.0);
        const double stopRamp = std::clamp((14.0 - time) / 1.0, 0.0, 1.0);
        const double velocity = 38.0 * ramp * stopRamp;
        const double omega = velocity == 0.0 ? 0.0
            : (0.42 * std::sin(0.72 * time) + (time > 8.0 ? 0.16 : 0.0));
        result.push_back({pose, {velocity, omega}});
        pose = stepUnicycle(pose, {velocity, omega}, kDt);
    }
    return result;
}

Horizon preview(const Horizon& full, std::size_t start, std::size_t previewLen) {
    Horizon result;
    result.reserve(previewLen + 1);
    for (std::size_t i = 0; i <= previewLen; ++i)
        result.push_back(full[std::min(start + i, full.size() - 1)]);
    return result;
}

struct ScenarioResult {
    double positionRmse = 0.0;
    double meanJerk = 0.0; // same quantity path_following_benchmark.cpp reports as mean_du
};

ScenarioResult runScenario(const RamseteLQRConfig& config, const Horizon& reference, const Scenario& scenario) {
    RamseteLQRController controller(config);
    State state{{reference.front().pose.x - 5.0, reference.front().pose.y + 7.0,
                 reference.front().pose.theta + 18.0 * std::numbers::pi / 180.0}, 0.0, 0.0};
    double positionSquares = 0.0;
    double jerkSum = 0.0;
    Control previousCommand{};
    // Fixed seed, independent of the candidate config, so every candidate is
    // scored against the exact same noise realization - otherwise a
    // "quieter" random draw could make a worse config look better by luck.
    Xorshift32 noiseRng(0xBEEF);

    for (std::size_t i = 0; i + 1 < reference.size(); ++i) {
        if (i == 105) state.pose.y += scenario.shoveInches;
        const Horizon local = preview(reference, i, scenario.previewLen);

        // The controller only ever sees a (possibly noisy) measurement of
        // the true state - modeling real odometry, which is never exact.
        // The plant below still evolves from the *true* state, so noise
        // here shows up as steering error, not a free pass on the metric.
        State measured = state;
        if (scenario.positionNoiseIn > 0.0) {
            measured.pose.x += scenario.positionNoiseIn * noiseRng.gaussian();
            measured.pose.y += scenario.positionNoiseIn * noiseRng.gaussian();
        }
        if (scenario.headingNoiseRad > 0.0) {
            measured.pose.theta += scenario.headingNoiseRad * noiseRng.gaussian();
        }
        const Control command = controller.calculate(measured, local, kDt);
        jerkSum += std::hypot(command.linear - previousCommand.linear, command.angular - previousCommand.angular);
        previousCommand = command;

        const double alpha = 1.0 - std::exp(-kDt / scenario.lagTau);
        state.linearVelocity += alpha * ((1.0 - scenario.speedLoss) * command.linear - state.linearVelocity);
        state.angularVelocity += alpha * (command.angular - state.angularVelocity);
        state.pose = stepUnicycle(state.pose, {state.linearVelocity, state.angularVelocity}, kDt);

        const auto error = poseError(state.pose, reference[i + 1].pose);
        const double position = std::hypot(error[0], error[1]);
        positionSquares += position * position;
    }
    const double count = static_cast<double>(reference.size() - 1);
    return {std::sqrt(positionSquares / count), jerkSum / count};
}

// Mean + std dev of per-scenario position RMSE (same "consistency across
// conditions matters as much as raw accuracy" framing as the notebook's
// cross-experiment table), plus a mean command-jerk term. The jerk term is
// what actually stops the search from just walking every gain to infinity:
// on the noise_scenario especially, an overly stiff gain reacts hard to
// every noisy measurement, and that shows up as jerk even in the (fairly
// common, see sensor_noise scenario above) case where the plant's own
// lag mostly filters it back out of position error - jerk is what a driver
// would actually feel as "twitchy" and what real motors/gearboxes pay for
// in wear and current draw, so it belongs in the cost even when position
// RMSE alone doesn't fully price it in.
struct Score {
    double mean = 0.0;
    double stddev = 0.0;
    double worst = 0.0;
    double jerkMean = 0.0;
    double combined = 0.0; // what the search actually optimizes
};

// Calibrated from the shipped defaults' own baseline jerk (~0.4-0.6, see
// path_following_notebook.md Experiment 1's ramsete_lqr row) so this term is
// comparable in scale to the position-RMSE terms above, not negligible and
// not dominant.
constexpr double kJerkWeight = 3.0;

Score scoreConfig(const RamseteLQRConfig& config, const Horizon& reference) {
    double positionValues[std::size(kScenarios)];
    double jerkSum = 0.0;
    for (std::size_t i = 0; i < std::size(kScenarios); ++i) {
        const ScenarioResult result = runScenario(config, reference, kScenarios[i]);
        positionValues[i] = result.positionRmse;
        jerkSum += result.meanJerk;
    }
    Score score;
    for (double v : positionValues) score.mean += v;
    score.mean /= std::size(kScenarios);
    double variance = 0.0;
    for (double v : positionValues) {
        variance += (v - score.mean) * (v - score.mean);
        score.worst = std::max(score.worst, v);
    }
    score.stddev = std::sqrt(variance / std::size(kScenarios));
    score.jerkMean = jerkSum / std::size(kScenarios);
    score.combined = score.mean + score.stddev + kJerkWeight * score.jerkMean;
    return score;
}

struct Candidate {
    RamseteLQRConfig config;
    Score score;
};

}

int main() {
    const Horizon reference = makeReference();
    constexpr std::uint32_t kSeed = 0x14683;
    constexpr int kSamples = 20000;
    Xorshift32 rng(kSeed);

    RamseteLQRConfig shipped{}; // current defaults, for comparison
    const Score shippedScore = scoreConfig(shipped, reference);

    std::vector<Candidate> top;
    top.reserve(6);

    for (int i = 0; i < kSamples; ++i) {
        RamseteLQRConfig candidate;
        candidate.b = rng.logUniform(0.00013, 0.02);
        candidate.zeta = rng.uniform(0.3, 0.95);
        candidate.velocityTimeConstant = rng.logUniform(0.03, 0.35);
        candidate.linearVelocityTolerance = rng.logUniform(0.3, 6.0);
        candidate.angularVelocityTolerance = rng.logUniform(0.02, 2.0);
        candidate.linearCommandTolerance = rng.logUniform(4.0, 30.0);
        candidate.angularCommandTolerance = rng.logUniform(1.0, 8.0);

        const Score score = scoreConfig(candidate, reference);
        if (!std::isfinite(score.combined)) continue;

        if (top.size() < top.capacity() || score.combined < top.back().score.combined) {
            top.push_back({candidate, score});
            std::sort(top.begin(), top.end(),
                     [](const Candidate& a, const Candidate& b) { return a.score.combined < b.score.combined; });
            if (top.size() > 6) top.pop_back();
        }
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "shipped defaults: b=" << shipped.b << " zeta=" << shipped.zeta
              << " tau=" << shipped.velocityTimeConstant
              << " lvTol=" << shipped.linearVelocityTolerance
              << " avTol=" << shipped.angularVelocityTolerance
              << " lcTol=" << shipped.linearCommandTolerance
              << " acTol=" << shipped.angularCommandTolerance << '\n';
    std::cout << "  mean=" << shippedScore.mean << " stddev=" << shippedScore.stddev
              << " worst=" << shippedScore.worst << " jerk=" << shippedScore.jerkMean
              << " combined=" << shippedScore.combined << "\n\n";

    std::cout << kSamples << " random samples, top " << top.size() << " by mean+stddev pos_rmse:\n\n";
    int rank = 1;
    for (const Candidate& c : top) {
        std::cout << "#" << rank++ << "  b=" << c.config.b << " zeta=" << c.config.zeta
                  << " tau=" << c.config.velocityTimeConstant
                  << " lvTol=" << c.config.linearVelocityTolerance
                  << " avTol=" << c.config.angularVelocityTolerance
                  << " lcTol=" << c.config.linearCommandTolerance
                  << " acTol=" << c.config.angularCommandTolerance << '\n';
        std::cout << "    mean=" << c.score.mean << " stddev=" << c.score.stddev
                  << " worst=" << c.score.worst << " jerk=" << c.score.jerkMean
                  << " combined=" << c.score.combined << '\n';
    }

    if (!top.empty() && top.front().score.combined < shippedScore.combined) {
        const double improvementPct = 100.0 * (shippedScore.combined - top.front().score.combined) / shippedScore.combined;
        std::cout << "\nBest candidate improves combined score by " << std::setprecision(1)
                  << improvementPct << "% over shipped defaults.\n";
    } else {
        std::cout << "\nNo sampled candidate beat the shipped defaults on this scenario set.\n";
    }
}
