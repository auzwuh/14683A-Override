// PC check for include/imu_fusion.hpp. Run:
//   g++ -std=c++20 -I include tests/imu_fusion_check.cpp -o imu_fusion_check && ./imu_fusion_check
#include "imu_fusion.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using imu_fusion::Fuser;
using imu_fusion::Sample;

static int failures = 0;
static void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

int main() {
    // Both sensors perfect: ten full turns in 1 deg steps adds up to 3600.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double r = 0;
        for (int i = 1; i <= 3600; ++i) { s[0].rawDeg = s[1].rawDeg = i; r = f.step(s); }
        check(near(r, 3600, 1e-9), "two perfect sensors give 3600 after ten turns");
    }

    // Mounted opposite ways, the same drift shows up with opposite signs and cancels.
    // 0.02 deg/s drift for 60 s of 10 ms updates, robot standing still.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double r = 0;
        for (int i = 1; i <= 6000; ++i) {
            s[0].rawDeg = 0.02 * i * 0.01;
            s[1].rawDeg = -0.02 * i * 0.01;
            r = f.step(s);
        }
        check(near(r, 0, 1e-9), "opposite drift cancels (single sensor would be off 1.2 deg)");
    }

    // Scale. Sensor A reads 1% high, B 2% low. Scales put both right.
    {
        Fuser f({.scale = {1.0 / 1.01, 1.0 / 0.98}});
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double r = 0;
        for (int i = 1; i <= 900; ++i) { s[0].rawDeg = 1.01 * i; s[1].rawDeg = 0.98 * i; r = f.step(s); }
        check(near(r, 900, 1e-6), "per-sensor scale corrects each sensor");
    }

    // B unplugged halfway through a turn. No jump, A carries on alone.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double before = 0, after = 0;
        for (int i = 1; i <= 180; ++i) {
            s[0].rawDeg = i;
            if (i <= 90) s[1].rawDeg = i; else s[1] = {false, INFINITY};
            const double r = f.step(s);
            if (i == 90) before = r;
            if (i == 91) after = r;
        }
        check(near(after - before, 1, 1e-9), "no jump when a sensor drops out");
        check(near(f.rotation(), 180, 1e-9), "remaining sensor keeps counting");
        check(!f.inUse(1) && f.inUse(0) && f.activeCount() == 1, "dropped sensor is out");
        // Plugged back in (recalibrated while moving, so reads from 0 again): stays out.
        s[1] = {true, 0};
        s[0].rawDeg = 181;
        f.step(s);
        check(!f.inUse(1) && near(f.rotation(), 181, 1e-9), "reconnected sensor stays out until next calibration");
    }

    // One-update spike on B (static shock) is ignored, not averaged in.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double r = 0;
        for (int i = 1; i <= 100; ++i) {
            s[0].rawDeg = i * 0.5;
            s[1].rawDeg = i * 0.5 + (i >= 50 ? 30.0 : 0.0);  // jumps 30 deg once, then normal
            r = f.step(s);
        }
        check(near(r, 50, 1e-9), "one-update 30 deg spike ignored");
        check(f.inUse(1), "one spike does not drop the sensor");
    }

    // B goes bad for good (reads double). Odd one out for 25 updates, then dropped.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        double r = 0;
        for (int i = 1; i <= 200; ++i) {
            s[0].rawDeg = i * 3.0;
            s[1].rawDeg = i <= 100 ? i * 3.0 : 300 + (i - 100) * 6.0;
            r = f.step(s);
        }
        check(near(r, 600, 1e-9), "bad sensor's readings never used");
        check(!f.inUse(1), "persistently bad sensor dropped");
    }

    // Missing at calibration: never used that run, even if it shows up later.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {false, INFINITY}};
        f.restart(s);
        check(f.activeCount() == 1 && !f.inUse(1), "sensor missing at calibration not used");
        s[1] = {true, 0};
        s[0].rawDeg = 10;
        check(near(f.step(s), 10, 1e-9) && !f.inUse(1), "late sensor ignored");
    }

    // Both gone: no heading (NaN), so odometry treats it like an unplugged IMU.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        s[0] = s[1] = {false, INFINITY};
        check(std::isnan(f.step(s)) && std::isnan(f.rotation()), "no sensors left gives NaN");
    }

    // setRotation (tare / set_heading) moves the fused value without disturbing tracking.
    {
        Fuser f;
        Sample s[2] = {{true, 0}, {true, 0}};
        f.restart(s);
        s[0].rawDeg = s[1].rawDeg = 45;
        f.step(s);
        f.setRotation(90);
        s[0].rawDeg = s[1].rawDeg = 50;
        check(near(f.step(s), 95, 1e-9), "setRotation then keeps counting from there");
    }

    if (failures == 0) std::printf("imu_fusion ok\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
