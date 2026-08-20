#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "gen/path_following/types.hpp"

namespace arc::path::detail {

constexpr std::size_t NX = 5;
constexpr std::size_t NU = 2;
using X = std::array<double, NX>;
using U = std::array<double, NU>;
using A = std::array<std::array<double, NX>, NX>;
using B = std::array<std::array<double, NU>, NX>;
using K = std::array<std::array<double, NX>, NU>;
using Mxx = A;
using Muu = std::array<std::array<double, NU>, NU>;

struct Gains {
    U feedforward{};
    K feedback{};
};

inline X makeState(const Pose& pose, const Control& previous) {
    return {pose.x, pose.y, pose.theta, previous.linear, previous.angular};
}

inline Pose poseOf(const X& state) { return {state[0], state[1], state[2]}; }
inline Control controlOf(const U& input) { return {input[0], input[1]}; }
inline U arrayOf(const Control& input) { return {input.linear, input.angular}; }

inline X dynamics(const X& state, const U& input, double dt) {
    const Pose next = stepUnicycle(poseOf(state), controlOf(input), dt);
    return {next.x, next.y, next.theta, input[0], input[1]};
}

inline void linearize(const X& state, const U& input, double dt, A& a, B& b) {
    constexpr double eps = 1e-5;
    for (std::size_t column = 0; column < NX; ++column) {
        X plus = state;
        X minus = state;
        plus[column] += eps;
        minus[column] -= eps;
        const X fp = dynamics(plus, input, dt);
        const X fm = dynamics(minus, input, dt);
        for (std::size_t row = 0; row < NX; ++row) {
            double difference = fp[row] - fm[row];
            if (row == 2) difference = wrapAngle(difference);
            a[row][column] = difference / (2.0 * eps);
        }
    }
    for (std::size_t column = 0; column < NU; ++column) {
        U plus = input;
        U minus = input;
        plus[column] += eps;
        minus[column] -= eps;
        const X fp = dynamics(state, plus, dt);
        const X fm = dynamics(state, minus, dt);
        for (std::size_t row = 0; row < NX; ++row) {
            double difference = fp[row] - fm[row];
            if (row == 2) difference = wrapAngle(difference);
            b[row][column] = difference / (2.0 * eps);
        }
    }
}

inline double cost(const X& initial, const std::vector<U>& controls, const Horizon& reference,
                   const CostWeights& weights, double dt, std::vector<X>* states = nullptr) {
    X state = initial;
    if (states) {
        states->clear();
        states->push_back(state);
    }
    double total = 0.0;
    const std::size_t count = std::min(controls.size(), reference.size() - 1);
    for (std::size_t i = 0; i < count; ++i) {
        const auto e = poseError(poseOf(state), reference[i].pose);
        const double eu0 = controls[i][0] - reference[i].velocity.linear;
        const double eu1 = controls[i][1] - reference[i].velocity.angular;
        const double er0 = controls[i][0] - state[3];
        const double er1 = controls[i][1] - state[4];
        for (std::size_t j = 0; j < 3; ++j) total += weights.state[j] * e[j] * e[j];
        total += weights.control[0] * eu0 * eu0 + weights.control[1] * eu1 * eu1;
        total += weights.controlRate[0] * er0 * er0 + weights.controlRate[1] * er1 * er1;
        state = dynamics(state, controls[i], dt);
        if (states) states->push_back(state);
    }
    const auto e = poseError(poseOf(state), reference[std::min(count, reference.size() - 1)].pose);
    for (std::size_t j = 0; j < 3; ++j) total += weights.terminal[j] * e[j] * e[j];
    return total;
}

inline bool inverse2(const Muu& input, Muu& inverse) {
    const double determinant = input[0][0] * input[1][1] - input[0][1] * input[1][0];
    if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12) return false;
    inverse = {{{input[1][1] / determinant, -input[0][1] / determinant},
                {-input[1][0] / determinant, input[0][0] / determinant}}};
    return true;
}

inline std::vector<U> initialGuess(const Horizon& reference, const std::vector<Control>& warm) {
    const std::size_t count = reference.size() > 1 ? reference.size() - 1 : 0;
    std::vector<U> result(count);
    for (std::size_t i = 0; i < count; ++i) {
        const Control value = i < warm.size() ? warm[i] : reference[i].velocity;
        result[i] = arrayOf(value);
    }
    return result;
}

inline std::vector<Control> optimize(const Pose& initialPose, const Control& previous,
                                     const Horizon& reference, const CostWeights& weights,
                                     const Limits& limits, double dt, std::size_t iterations,
                                     double regularization, const std::vector<Control>& warm) {
    if (reference.size() < 2 || !(dt > 0.0)) return {};
    const X initial = makeState(initialPose, previous);
    std::vector<U> controls = initialGuess(reference, warm);

    // Clamp the warm start into the same feasible set used by forward passes.
    Control prior = previous;
    for (U& input : controls) {
        const Control bounded = clampControl(controlOf(input), prior, limits, dt);
        input = arrayOf(bounded);
        prior = bounded;
    }

    std::vector<X> states;
    double bestCost = cost(initial, controls, reference, weights, dt, &states);
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        const std::size_t count = controls.size();
        std::vector<Gains> gains(count);
        X vx{};
        Mxx vxx{};
        const auto terminalError = poseError(poseOf(states.back()), reference[count].pose);
        for (std::size_t i = 0; i < 3; ++i) {
            vx[i] = 2.0 * weights.terminal[i] * terminalError[i];
            vxx[i][i] = 2.0 * weights.terminal[i];
        }

        bool valid = true;
        for (std::size_t reverse = count; reverse-- > 0;) {
            A a{};
            B b{};
            linearize(states[reverse], controls[reverse], dt, a, b);
            const auto error = poseError(poseOf(states[reverse]), reference[reverse].pose);
            const U uref = arrayOf(reference[reverse].velocity);
            const U rateError{controls[reverse][0] - states[reverse][3],
                              controls[reverse][1] - states[reverse][4]};
            X lx{};
            U lu{};
            Mxx lxx{};
            Muu luu{};
            std::array<std::array<double, NX>, NU> lux{};
            for (std::size_t i = 0; i < 3; ++i) {
                lx[i] = 2.0 * weights.state[i] * error[i];
                lxx[i][i] = 2.0 * weights.state[i];
            }
            for (std::size_t i = 0; i < NU; ++i) {
                lx[3 + i] = -2.0 * weights.controlRate[i] * rateError[i];
                lxx[3 + i][3 + i] = 2.0 * weights.controlRate[i];
                lu[i] = 2.0 * weights.control[i] * (controls[reverse][i] - uref[i]) +
                        2.0 * weights.controlRate[i] * rateError[i];
                luu[i][i] = 2.0 * (weights.control[i] + weights.controlRate[i]);
                lux[i][3 + i] = -2.0 * weights.controlRate[i];
            }

            X qx = lx;
            U qu = lu;
            Mxx qxx = lxx;
            Muu quu = luu;
            std::array<std::array<double, NX>, NU> qux = lux;
            for (std::size_t i = 0; i < NX; ++i) {
                for (std::size_t j = 0; j < NX; ++j) {
                    double av = 0.0;
                    for (std::size_t k = 0; k < NX; ++k) av += a[k][i] * vxx[k][j];
                    for (std::size_t k = 0; k < NX; ++k) qxx[i][k] += av * a[j][k];
                }
                for (std::size_t k = 0; k < NX; ++k) qx[i] += a[k][i] * vx[k];
            }
            for (std::size_t i = 0; i < NU; ++i) {
                for (std::size_t k = 0; k < NX; ++k) qu[i] += b[k][i] * vx[k];
                for (std::size_t j = 0; j < NU; ++j) {
                    for (std::size_t m = 0; m < NX; ++m) {
                        for (std::size_t n = 0; n < NX; ++n) quu[i][j] += b[m][i] * vxx[m][n] * b[n][j];
                    }
                }
                for (std::size_t j = 0; j < NX; ++j) {
                    for (std::size_t m = 0; m < NX; ++m) {
                        for (std::size_t n = 0; n < NX; ++n) qux[i][j] += b[m][i] * vxx[m][n] * a[n][j];
                    }
                }
            }
            quu[0][0] += regularization;
            quu[1][1] += regularization;
            Muu inverse{};
            if (!inverse2(quu, inverse)) {
                valid = false;
                break;
            }
            Gains& gain = gains[reverse];
            for (std::size_t i = 0; i < NU; ++i) {
                for (std::size_t j = 0; j < NU; ++j) {
                    gain.feedforward[i] -= inverse[i][j] * qu[j];
                    for (std::size_t k = 0; k < NX; ++k) gain.feedback[i][k] -= inverse[i][j] * qux[j][k];
                }
            }

            // Value update: Vx = Qx + K'Quu*k + K'Qu + Qux'*k
            // Vxx = Qxx + K'Quu*K + K'Qux + Qux'*K.
            X nextVx = qx;
            Mxx nextVxx = qxx;
            for (std::size_t i = 0; i < NX; ++i) {
                for (std::size_t aidx = 0; aidx < NU; ++aidx) {
                    nextVx[i] += gain.feedback[aidx][i] * qu[aidx] + qux[aidx][i] * gain.feedforward[aidx];
                    for (std::size_t bidx = 0; bidx < NU; ++bidx) {
                        nextVx[i] += gain.feedback[aidx][i] * quu[aidx][bidx] * gain.feedforward[bidx];
                    }
                    for (std::size_t j = 0; j < NX; ++j) {
                        nextVxx[i][j] += gain.feedback[aidx][i] * qux[aidx][j] +
                                         qux[aidx][i] * gain.feedback[aidx][j];
                        for (std::size_t bidx = 0; bidx < NU; ++bidx) {
                            nextVxx[i][j] += gain.feedback[aidx][i] * quu[aidx][bidx] * gain.feedback[bidx][j];
                        }
                    }
                }
            }
            vx = nextVx;
            vxx = nextVxx;
        }
        if (!valid) break;

        bool accepted = false;
        for (double alpha : {1.0, 0.5, 0.25, 0.1, 0.05}) {
            X trialState = initial;
            std::vector<U> trial(count);
            Control trialPrior = previous;
            for (std::size_t i = 0; i < count; ++i) {
                U candidate = controls[i];
                for (std::size_t row = 0; row < NU; ++row) {
                    candidate[row] += alpha * gains[i].feedforward[row];
                    for (std::size_t column = 0; column < NX; ++column) {
                        double delta = trialState[column] - states[i][column];
                        if (column == 2) delta = wrapAngle(delta);
                        candidate[row] += gains[i].feedback[row][column] * delta;
                    }
                }
                const Control bounded = clampControl(controlOf(candidate), trialPrior, limits, dt);
                trial[i] = arrayOf(bounded);
                trialPrior = bounded;
                trialState = dynamics(trialState, trial[i], dt);
            }
            const double trialCost = cost(initial, trial, reference, weights, dt);
            if (trialCost + 1e-9 < bestCost) {
                controls = std::move(trial);
                bestCost = trialCost;
                cost(initial, controls, reference, weights, dt, &states);
                accepted = true;
                break;
            }
        }
        if (!accepted) break;
    }

    std::vector<Control> result;
    result.reserve(controls.size());
    for (const U& input : controls) result.push_back(controlOf(input));
    return result;
}

inline std::vector<Control> shiftWarmStart(const std::vector<Control>& solution, const Horizon& reference) {
    if (solution.empty()) return {};
    std::vector<Control> result(solution.size());
    for (std::size_t i = 0; i + 1 < solution.size(); ++i) result[i] = solution[i + 1];
    const std::size_t tail = solution.size() - 1;
    result[tail] = tail + 1 < reference.size() ? reference[tail + 1].velocity : solution.back();
    return result;
}

} // namespace arc::path::detail
