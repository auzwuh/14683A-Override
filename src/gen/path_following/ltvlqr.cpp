#include "gen/path_following/ltvlqr.hpp"

#include <array>

namespace arc::path {
namespace {

using M3 = std::array<std::array<double, 3>, 3>;
using B32 = std::array<std::array<double, 2>, 3>;
using M2 = std::array<std::array<double, 2>, 2>;
using K23 = std::array<std::array<double, 3>, 2>;

bool inverse(const M2& matrix, M2& result) {
    const double determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if (std::abs(determinant) < 1e-12) return false;
    result = {{{matrix[1][1] / determinant, -matrix[0][1] / determinant},
               {-matrix[1][0] / determinant, matrix[0][0] / determinant}}};
    return true;
}

void linearize(const Reference& reference, double dt, M3& a, B32& b) {
    // Jacobian of exact ZOH unicycle integration, evaluated numerically to
    // remain well-conditioned through omega=0 and curved reference segments.
    constexpr double eps = 1e-5;
    for (std::size_t column = 0; column < 3; ++column) {
        Pose plus = reference.pose;
        Pose minus = reference.pose;
        (&plus.x)[column] += eps;
        (&minus.x)[column] -= eps;
        const Pose fp = stepUnicycle(plus, reference.velocity, dt);
        const Pose fm = stepUnicycle(minus, reference.velocity, dt);
        const std::array<double, 3> difference{fp.x - fm.x, fp.y - fm.y,
                                               wrapAngle(fp.theta - fm.theta)};
        for (std::size_t row = 0; row < 3; ++row) a[row][column] = difference[row] / (2.0 * eps);
    }
    for (std::size_t column = 0; column < 2; ++column) {
        Control plus = reference.velocity;
        Control minus = reference.velocity;
        (&plus.linear)[column] += eps;
        (&minus.linear)[column] -= eps;
        const Pose fp = stepUnicycle(reference.pose, plus, dt);
        const Pose fm = stepUnicycle(reference.pose, minus, dt);
        const std::array<double, 3> difference{fp.x - fm.x, fp.y - fm.y,
                                               wrapAngle(fp.theta - fm.theta)};
        for (std::size_t row = 0; row < 3; ++row) b[row][column] = difference[row] / (2.0 * eps);
    }
}

} // namespace

LTVLQRController::LTVLQRController(CostWeights weights, Limits limits)
    : weights_(weights), limits_(limits) {}

Control LTVLQRController::calculate(const State& state, const Horizon& reference, double dt) {
    if (reference.size() < 2 || !(dt > 0.0)) return {};
    M3 p{};
    for (std::size_t i = 0; i < 3; ++i) p[i][i] = weights_.terminal[i];
    K23 firstGain{};

    for (std::size_t reverse = reference.size() - 1; reverse-- > 0;) {
        M3 a{};
        B32 b{};
        linearize(reference[reverse], dt, a, b);
        M2 s{};
        K23 btpa{};
        for (std::size_t i = 0; i < 2; ++i) {
            s[i][i] = weights_.control[i];
            for (std::size_t j = 0; j < 2; ++j) {
                for (std::size_t m = 0; m < 3; ++m)
                    for (std::size_t n = 0; n < 3; ++n) s[i][j] += b[m][i] * p[m][n] * b[n][j];
            }
            for (std::size_t j = 0; j < 3; ++j)
                for (std::size_t m = 0; m < 3; ++m)
                    for (std::size_t n = 0; n < 3; ++n) btpa[i][j] += b[m][i] * p[m][n] * a[n][j];
        }
        M2 sinverse{};
        if (!inverse(s, sinverse)) return reference.front().velocity;
        K23 gain{};
        for (std::size_t i = 0; i < 2; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                for (std::size_t k = 0; k < 2; ++k) gain[i][j] += sinverse[i][k] * btpa[k][j];

        M3 next{};
        for (std::size_t i = 0; i < 3; ++i) {
            next[i][i] = weights_.state[i];
            for (std::size_t j = 0; j < 3; ++j) {
                for (std::size_t m = 0; m < 3; ++m) {
                    double closedLoop = a[m][j];
                    for (std::size_t u = 0; u < 2; ++u) closedLoop -= b[m][u] * gain[u][j];
                    for (std::size_t n = 0; n < 3; ++n) next[i][j] += a[n][i] * p[n][m] * closedLoop;
                }
            }
        }
        // Roundoff can otherwise slowly destroy the Riccati matrix symmetry.
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = i + 1; j < 3; ++j) next[i][j] = next[j][i] = 0.5 * (next[i][j] + next[j][i]);
        p = next;
        if (reverse == 0) firstGain = gain;
    }

    const auto error = poseError(state.pose, reference.front().pose);
    Control command = reference.front().velocity;
    for (std::size_t i = 0; i < 3; ++i) {
        command.linear -= firstGain[0][i] * error[i];
        command.angular -= firstGain[1][i] * error[i];
    }
    previous_ = clampControl(command, previous_, limits_, dt);
    return previous_;
}

void LTVLQRController::reset(Control previous) { previous_ = previous; }

} // namespace arc::path
