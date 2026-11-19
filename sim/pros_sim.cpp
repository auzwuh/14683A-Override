// pros_sim.cpp - the PROS kernel and device API the robot code links against.
//
// Scheduler. The V5 runs FreeRTOS on one core and robot code only gives the CPU
// away inside delay() or a blocking mutex take. Here every pros::Task is a host
// thread, but a single baton (cur) lets exactly one run. delay(ms) parks the task
// until virtual time now+ms and hands the baton back to the kernel loop. When no
// task is runnable the kernel steps physics 1 ms and advances virtual time.
// Ready tasks run in (wake time, time they parked) order: equal priority round
// robin, deterministic. Everything is priority 8 on this robot so priorities are
// ignored. ponytail: cooperative, no 1 ms preemption; robot slices run in
// microseconds so a tick-preempting scheduler would give the same order.
// LIMIT: a slice between two delay() calls is atomic and takes zero virtual time, so
// this sim cannot show data races between tasks (unprotected shared state in the robot
// code) or a task overrunning its period. "No crash or hang" here does not cover those.
#include "api.h"
#include "sim_core.hpp"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <system_error>
#include <thread>

namespace sim {
namespace {
struct SMutex;
struct STask {
    int id = 0;
    std::string name;
    std::function<void()> fn;
    uint64_t wake = 0, seq = 0;
    bool done = false, killed = false;
    SMutex* waitM = nullptr;
    std::condition_variable cv;
};
struct SMutex {
    STask* owner = nullptr;
    bool kernelOwned = false;
    std::deque<STask*> waiters;
};
constexpr uint64_t kInf = ~0ull;
std::mutex big;
std::condition_variable kcv;
STask* cur = nullptr;
std::vector<STask*> tasks;
uint64_t vnow = 0, seqc = 0;
thread_local STask* self = nullptr;
// Host locks and waits may touch errno; robot odometry treats errno != 0 after a sensor read as a
// failed read (odom.cpp:35-53), so the kernel never lets its own errno leak into a task.
struct KeepErrno { int e = errno; ~KeepErrno() { errno = e; } };

void parkLocked(std::unique_lock<std::mutex>& lk, STask* t) {
    cur = nullptr;
    kcv.notify_all();
    t->cv.wait(lk, [t] { return cur == t; });
}
void sleepUntil(uint64_t wake) {
    KeepErrno keep;
    STask* t = self;
    if (!t) {
        std::fprintf(stderr, "[sim] FATAL delay() called outside a task (static init or kernel)\n");
        std::_Exit(5);
    }
    std::unique_lock<std::mutex> lk(big);
    t->wake = wake;
    t->seq = ++seqc;
    parkLocked(lk, t);
}
}  // namespace

uint64_t now() { return vnow; }
std::string currentTaskName() { return self ? self->name : std::string("kernel"); }

int spawn(std::function<void()> fn, const std::string& name) {
    KeepErrno keep;
    std::unique_lock<std::mutex> lk(big);
    auto* t = new STask;
    t->id = static_cast<int>(tasks.size());
    t->name = name.empty() ? "task" + std::to_string(t->id) : name;
    t->fn = std::move(fn);
    t->wake = vnow;
    t->seq = ++seqc;
    tasks.push_back(t);
    std::thread([t] {
        self = t;
        {
            std::unique_lock<std::mutex> l(big);
            t->cv.wait(l, [t] { return cur == t; });
        }
        t->fn();
        std::unique_lock<std::mutex> l(big);
        t->done = true;
        cur = nullptr;
        kcv.notify_all();
    }).detach();
    return t->id;
}

void killTask(int id) {
    std::unique_lock<std::mutex> lk(big);
    STask* t = tasks.at(id);
    if (t->done) return;
    t->killed = true;   // its thread stays parked forever, like a deleted FreeRTOS task
    if (t->waitM) {     // FreeRTOS takes a deleted task off the mutex's wait list
        auto& w = t->waitM->waiters;
        w.erase(std::remove(w.begin(), w.end(), t), w.end());
        t->waitM = nullptr;
    }
}
bool taskDone(int id) {
    std::unique_lock<std::mutex> lk(big);
    return tasks.at(id)->done;
}

[[noreturn]] void kernelRun(const std::function<void(uint64_t)>& stepMs, const std::function<void(uint64_t)>& hook) {
    using clk = std::chrono::steady_clock;
    std::unique_lock<std::mutex> lk(big);
    while (true) {
        STask* best = nullptr;
        for (STask* t : tasks) {
            if (t->done || t->killed || t->wake > vnow) continue;
            if (!best || t->wake < best->wake || (t->wake == best->wake && t->seq < best->seq)) best = t;
        }
        if (best) {
            cur = best;
            best->cv.notify_all();
            const auto t0 = clk::now();
            while (cur != nullptr) {
                kcv.wait_for(lk, std::chrono::milliseconds(200));
                if (cur != nullptr && clk::now() - t0 > std::chrono::milliseconds(opt.watchdogMs)) {
                    std::fprintf(stderr,
                                 "\n[sim] WATCHDOG: task '%s' ran %d ms of real time without yielding at virtual t=%llu ms. "
                                 "On a V5 a task that never calls delay() starves every lower-priority task and takes "
                                 "every other tick from tasks of its own priority.\n",
                                 cur->name.c_str(), opt.watchdogMs, static_cast<unsigned long long>(vnow));
                    std::fflush(stderr);
                    logLine("[sim] WATCHDOG fired in task '" + cur->name + "'");
                    std::_Exit(3);
                }
            }
            continue;
        }
        lk.unlock();
        stepMs(vnow);   // physics vnow -> vnow+1
        lk.lock();
        ++vnow;
        lk.unlock();
        hook(vnow);
        lk.lock();
    }
}

// ------------------------------------------------------------- mutex helpers
namespace {
SMutex* sm(void* p) { return static_cast<SMutex*>(p); }
bool takeM(SMutex* s, uint32_t timeout) {
    KeepErrno keep;
    std::unique_lock<std::mutex> lk(big);
    STask* t = self;
    if (!t) {   // kernel thread (logging reads the odometry pose) or static init
        if (!s->owner && !s->kernelOwned) { s->kernelOwned = true; return true; }
        std::fprintf(stderr, "[sim] kernel could not take a held mutex\n");
        return false;
    }
    if (!s->owner && !s->kernelOwned) { s->owner = t; return true; }
    if (s->owner == t)
        logLine("[sim] WARNING task '" + t->name + "' took a non-recursive pros::Mutex it already holds (deadlocks on a V5)");
    if (timeout == 0) return false;
    s->waiters.push_back(t);
    t->waitM = s;
    t->wake = timeout == TIMEOUT_MAX ? kInf : vnow + timeout;
    t->seq = ++seqc;
    parkLocked(lk, t);
    t->waitM = nullptr;
    if (s->owner == t) return true;
    s->waiters.erase(std::remove(s->waiters.begin(), s->waiters.end(), t), s->waiters.end());
    return false;
}
void giveM(SMutex* s) {
    KeepErrno keep;
    std::unique_lock<std::mutex> lk(big);
    if (s->kernelOwned) { s->kernelOwned = false; return; }
    s->owner = nullptr;
    while (!s->waiters.empty() && (s->waiters.front()->killed || s->waiters.front()->done)) s->waiters.pop_front();
    if (!s->waiters.empty()) {
        STask* n = s->waiters.front();
        s->waiters.pop_front();
        s->owner = n;
        n->wake = vnow;
        n->seq = ++seqc;
    }
}
}  // namespace
}  // namespace sim

// ================================================================ C API
extern "C" {
uint32_t millis(void) { return static_cast<uint32_t>(sim::vnow); }
uint64_t micros(void) { return sim::vnow * 1000ull; }
void delay(const uint32_t ms) { sim::sleepUntil(sim::vnow + ms); }
void task_delay(const uint32_t ms) { sim::sleepUntil(sim::vnow + ms); }
int32_t controller_rumble(pros::controller_id_e_t, const char* p) {
    const int saved = errno;
    sim::logRumble(p);
    errno = saved;
    return 1;
}
int32_t controller_print(pros::controller_id_e_t, uint8_t, uint8_t, const char*, ...) { return 1; }   // no screen in the sim
double imu_get_rotation(uint8_t port) {
    auto& d = sim::imu(port);
    if (!d.exists) { errno = ENODEV; return PROS_ERR_F; }
    if (d.calibrating) { errno = EAGAIN; return PROS_ERR_F; }
    sim::imuReadNote(port, d.raw - d.offset);
    return d.raw - d.offset;
}

// Robot printf goes here (sim_printf.h is force-included into robot sources).
int sim_printf(const char* f, ...) {
    static std::string partial;
    char buf[4096];
    va_list ap;
    va_start(ap, f);
    const int saved = errno;
    const int n = std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    partial += buf;
    size_t pos;
    while ((pos = partial.find('\n')) != std::string::npos) {
        sim::robotLine(sim::currentTaskName(), partial.substr(0, pos));
        partial.erase(0, pos + 1);
    }
    errno = saved;
    return n;
}
}

// ================================================================ C++ API
namespace pros { namespace competition { std::uint8_t is_connected(void) { return 0; } } }   // never under field control here
namespace pros {
inline namespace rtos {
Task::Task(task_fn_t function, void* parameters, std::uint32_t, std::uint16_t, const char* name) {
    const int id = sim::spawn([function, parameters] { function(parameters); }, name ? name : "");
    task = reinterpret_cast<task_t>(static_cast<intptr_t>(id + 1));
}
Task::Task(task_fn_t function, void* parameters, const char* name)
    : Task(function, parameters, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name) {}
void Task::delay(const std::uint32_t ms) { sim::sleepUntil(sim::vnow + ms); }
void Task::delay_until(std::uint32_t* const prev, const std::uint32_t delta) {
    *prev += delta;
    if (*prev > sim::vnow) sim::sleepUntil(*prev);
}
mutex_t Mutex::lazy_init() {
    mutex_t m = mutex.load();
    if (m) return m;
    auto* fresh = new sim::SMutex;
    if (mutex.compare_exchange_strong(m, fresh)) return fresh;
    delete fresh;
    return m;
}
bool Mutex::take(std::uint32_t timeout) { return sim::takeM(sim::sm(lazy_init()), timeout); }
bool Mutex::take() { return take(TIMEOUT_MAX); }
bool Mutex::give() { sim::giveM(sim::sm(lazy_init())); return true; }
void Mutex::lock() {
    if (!take(TIMEOUT_MAX)) throw std::system_error(errno, std::system_category(), "Cannot obtain lock!");
}
void Mutex::unlock() { give(); }
bool Mutex::try_lock() { return take(0); }
Mutex::~Mutex() {}
}  // namespace rtos

inline namespace v5 {
// ------------------------------------------------------------ Device
Device::Device(const std::uint8_t port) : _port(port) {}
bool Device::is_installed() { return true; }
std::uint8_t Device::get_port(void) const { return _port; }

// ------------------------------------------------------------ Motor (only constructed by robot code)
Motor::Motor(const std::int8_t port, const MotorGears gearset, const MotorUnits units)
    : Device(static_cast<std::uint8_t>(port < 0 ? -port : port)), _port(port) {
    sim::motorRegister(port, gearset == MotorGears::invalid ? -1 : static_cast<int>(gearset),
                       units == MotorUnits::invalid ? -1 : static_cast<int>(units));
}

// ------------------------------------------------------------ MotorGroup
static int ap(std::int8_t p) { return p < 0 ? -p : p; }
MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports, const MotorGears g, const MotorUnits u)
    : _ports(ports) {
    for (auto p : _ports) sim::motorRegister(p, g == MotorGears::invalid ? -1 : static_cast<int>(g), u == MotorUnits::invalid ? -1 : static_cast<int>(u));
}
MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears g, const MotorUnits u) : _ports(ports) {
    for (auto p : _ports) sim::motorRegister(p, g == MotorGears::invalid ? -1 : static_cast<int>(g), u == MotorUnits::invalid ? -1 : static_cast<int>(u));
}
MotorGroup::MotorGroup(AbstractMotor& m) : _ports(m.get_port_all()) {}

#define MG_INDEX_CHECK(ret)                                  \
    if (index >= _ports.size()) { errno = EOVERFLOW; return ret; }
std::int32_t MotorGroup::move(std::int32_t v) const {
    v = std::clamp<std::int32_t>(v, -127, 127);
    for (auto p : _ports) sim::motorCommand(ap(p), sim::kVolt, v * 12000 / 127);
    return 1;
}
std::int32_t MotorGroup::move_voltage(const std::int32_t mv) const {
    for (auto p : _ports) sim::motorCommand(ap(p), sim::kVolt, std::clamp<std::int32_t>(mv, -12000, 12000));
    return 1;
}
std::int32_t MotorGroup::move_velocity(const std::int32_t rpm) const {
    for (auto p : _ports) sim::motorCommand(ap(p), sim::kVel, rpm);
    return 1;
}
std::int32_t MotorGroup::move_relative(const double position, const std::int32_t velocity) const {
    for (auto p : _ports) sim::motorCommandRel(ap(p), position, velocity);
    return 1;
}
std::int32_t MotorGroup::brake(void) const {
    for (auto p : _ports) sim::motorCommand(ap(p), sim::kBrake, 0);
    return 1;
}
double MotorGroup::get_position(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR_F);
    return sim::motorReadPos(ap(_ports[index]));
}
std::vector<double> MotorGroup::get_position_all(void) const {
    std::vector<double> r;
    for (auto p : _ports) r.push_back(sim::motorReadPos(ap(p)));
    return r;
}
double MotorGroup::get_actual_velocity(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR_F);
    return sim::motorReadRpm(ap(_ports[index]));
}
std::vector<double> MotorGroup::get_actual_velocity_all(void) const {
    std::vector<double> r;
    for (auto p : _ports) r.push_back(sim::motorReadRpm(ap(p)));
    return r;
}
std::int32_t MotorGroup::get_voltage(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    const auto& m = sim::motor(ap(_ports[index]));
    return static_cast<std::int32_t>((m.reversed ? -1 : 1) * m.volt * 1000.0);
}
std::int32_t MotorGroup::get_current_draw(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    const auto& m = sim::motor(ap(_ports[index]));
    return static_cast<std::int32_t>(std::fabs(m.torque) / m.tauLim * 2500.0);
}
double MotorGroup::get_torque(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR_F);
    const auto& m = sim::motor(ap(_ports[index]));
    return (m.reversed ? -1 : 1) * m.torque;
}
double MotorGroup::get_temperature(const std::uint8_t) const { return 30.0; }
std::vector<MotorGears> MotorGroup::get_gearing_all(void) const {
    std::vector<MotorGears> r;
    for (auto p : _ports) r.push_back(static_cast<MotorGears>(sim::motor(ap(p)).gear));
    return r;
}
MotorGears MotorGroup::get_gearing(const std::uint8_t index) const {
    MG_INDEX_CHECK(MotorGears::invalid);
    return static_cast<MotorGears>(sim::motor(ap(_ports[index])).gear);
}
std::int8_t MotorGroup::size(void) const { return static_cast<std::int8_t>(_ports.size()); }
std::int8_t MotorGroup::get_port(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR_BYTE);
    return _ports[index];
}
std::vector<std::int8_t> MotorGroup::get_port_all(void) const { return _ports; }
MotorBrake MotorGroup::get_brake_mode(const std::uint8_t index) const {
    MG_INDEX_CHECK(MotorBrake::invalid);
    return static_cast<MotorBrake>(sim::motor(ap(_ports[index])).brake);
}
std::vector<MotorBrake> MotorGroup::get_brake_mode_all(void) const {
    std::vector<MotorBrake> r;
    for (auto p : _ports) r.push_back(static_cast<MotorBrake>(sim::motor(ap(p)).brake));
    return r;
}
MotorUnits MotorGroup::get_encoder_units(const std::uint8_t index) const {
    MG_INDEX_CHECK(MotorUnits::invalid);
    return static_cast<MotorUnits>(sim::motor(ap(_ports[index])).units);
}
std::vector<MotorUnits> MotorGroup::get_encoder_units_all(void) const {
    std::vector<MotorUnits> r;
    for (auto p : _ports) r.push_back(static_cast<MotorUnits>(sim::motor(ap(p)).units));
    return r;
}
std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    sim::motor(ap(_ports[index])).brake = static_cast<int>(mode);
    return 1;
}
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const {
    return set_brake_mode(static_cast<MotorBrake>(mode), index);
}
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const {
    for (auto p : _ports) sim::motor(ap(p)).brake = static_cast<int>(mode);
    return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const {
    return set_brake_mode_all(static_cast<MotorBrake>(mode));
}
std::int32_t MotorGroup::set_encoder_units(const MotorUnits units, const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    sim::motor(ap(_ports[index])).units = static_cast<int>(units);
    return 1;
}
std::int32_t MotorGroup::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const {
    return set_encoder_units(static_cast<MotorUnits>(units), index);
}
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits units) const {
    for (auto p : _ports) sim::motor(ap(p)).units = static_cast<int>(units);
    return 1;
}
std::int32_t MotorGroup::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const {
    return set_encoder_units_all(static_cast<MotorUnits>(units));
}
std::int32_t MotorGroup::tare_position(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    sim::motorTare(ap(_ports[index]), 0.0);
    return 1;
}
std::int32_t MotorGroup::tare_position_all(void) const {
    for (auto p : _ports) sim::motorTare(ap(p), 0.0);
    return 1;
}
std::int32_t MotorGroup::set_zero_position(const double position, const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    sim::motorTare(ap(_ports[index]), position);
    return 1;
}
std::int32_t MotorGroup::set_zero_position_all(const double position) const {
    for (auto p : _ports) sim::motorTare(ap(p), position);
    return 1;
}
std::int32_t MotorGroup::is_reversed(const std::uint8_t index) const {
    MG_INDEX_CHECK(PROS_ERR);
    return sim::motor(ap(_ports[index])).reversed;
}
std::int32_t MotorGroup::set_reversed(const bool reverse, const std::uint8_t index) {
    MG_INDEX_CHECK(PROS_ERR);
    sim::motor(ap(_ports[index])).reversed = reverse;
    return 1;
}
std::int32_t MotorGroup::set_reversed_all(const bool reverse) {
    for (auto p : _ports) sim::motor(ap(p)).reversed = reverse;
    return 1;
}

// ------------------------------------------------------------ Rotation
Rotation::Rotation(const std::int8_t port) : Device(static_cast<std::uint8_t>(port < 0 ? -port : port)) {
    sim::rotationRegister(port);
}
static double rotRead(const sim::RotationDev& r) { return (r.reversed ? -1.0 : 1.0) * (r.sample - r.zero); }
std::int32_t Rotation::reset() {
    auto& r = sim::rotation(_port);
    const double a = std::fmod(rotRead(r), 36000.0);
    r.zero = r.sample - (r.reversed ? -1.0 : 1.0) * (a < 0 ? a + 36000.0 : a);
    return 1;
}
std::int32_t Rotation::set_data_rate(std::uint32_t) const { return 1; }
std::int32_t Rotation::set_position(std::int32_t pos) const {
    auto& r = sim::rotation(_port);
    r.zero = r.sample - (r.reversed ? -1.0 : 1.0) * pos;
    return 1;
}
std::int32_t Rotation::reset_position(void) const { return set_position(0); }
std::int32_t Rotation::get_position() const {
    auto& r = sim::rotation(_port);
    if (!r.exists) { errno = ENODEV; return PROS_ERR; }
    return static_cast<std::int32_t>(std::lround(rotRead(r)));
}
std::int32_t Rotation::get_velocity() const {
    auto& r = sim::rotation(_port);
    return static_cast<std::int32_t>(std::lround((r.reversed ? -1.0 : 1.0) * r.sampleVel));
}
std::int32_t Rotation::get_angle() const {
    double a = std::fmod(rotRead(sim::rotation(_port)), 36000.0);
    return static_cast<std::int32_t>(a < 0 ? a + 36000.0 : a);
}
std::int32_t Rotation::set_reversed(bool value) const {
    auto& r = sim::rotation(_port);
    if (value != r.reversed) {   // the reading keeps its magnitude, flips sign
        r.reversed = value;
    }
    return 1;
}
std::int32_t Rotation::reverse() const { return set_reversed(!sim::rotation(_port).reversed); }
std::int32_t Rotation::get_reversed() const { return sim::rotation(_port).reversed; }

// ------------------------------------------------------------ Distance
Distance::Distance(const std::uint8_t port) : Device(port) { sim::distanceRegister(port); }
std::int32_t Distance::get() { return sim::distanceGet(_port); }
std::int32_t Distance::get_distance() { return sim::distanceGet(_port); }
std::int32_t Distance::get_confidence() { return sim::distanceGet(_port) < 9999 ? 63 : 0; }
std::int32_t Distance::get_object_size() { return sim::distanceGet(_port) < 9999 ? 200 : -1; }
double Distance::get_object_velocity() { return 0.0; }

// ------------------------------------------------------------ Imu (ctor is inline in imu.hpp)
std::int32_t Imu::reset(bool blocking) const {
    if (!sim::imu(_port).exists) { errno = ENODEV; return PROS_ERR; }
    sim::imuReset(_port);
    if (blocking) while (sim::imu(_port).calibrating) pros::c::delay(10);
    return 1;
}
std::int32_t Imu::set_data_rate(std::uint32_t) const { return 1; }
double Imu::get_rotation() const { return pros::c::imu_get_rotation(_port); }
double Imu::get_heading() const {
    errno = 0;
    const double r = pros::c::imu_get_rotation(_port);
    if (errno) return PROS_ERR_F;
    const double h = std::fmod(r, 360.0);
    return h < 0 ? h + 360.0 : h;
}
double Imu::get_pitch() const { return !sim::imu(_port).exists || sim::imu(_port).calibrating ? PROS_ERR_F : sim::imu(_port).pitch; }
double Imu::get_roll() const { return !sim::imu(_port).exists || sim::imu(_port).calibrating ? PROS_ERR_F : sim::imu(_port).roll; }
double Imu::get_yaw() const {
    const double h = get_heading();
    return h > 180.0 ? h - 360.0 : h;
}
std::int32_t Imu::tare_rotation() const { return set_rotation(0); }
std::int32_t Imu::tare_heading() const { return set_heading(0); }
std::int32_t Imu::tare_yaw() const { return set_heading(0); }
std::int32_t Imu::tare() const { return set_rotation(0); }
std::int32_t Imu::set_rotation(const double target) const {
    auto& d = sim::imu(_port);
    d.offset = d.raw - target;
    return 1;
}
std::int32_t Imu::set_heading(const double target) const { return set_rotation(target); }
std::int32_t Imu::set_yaw(const double target) const { return set_rotation(target); }
pros::ImuStatus Imu::get_status() const {
    auto& d = sim::imu(_port);
    if (!d.exists) return pros::ImuStatus::error;
    return d.calibrating ? pros::ImuStatus::calibrating : pros::ImuStatus::ready;
}
bool Imu::is_calibrating() const { return sim::imu(_port).calibrating; }

// ------------------------------------------------------------ Controller
Controller::Controller(controller_id_e_t id) : _id(id) {}
std::int32_t Controller::get_analog(controller_analog_e_t ch) { return sim::controllerAnalog(static_cast<int>(ch)); }
std::int32_t Controller::get_digital(controller_digital_e_t b) { return sim::controllerDigital(static_cast<int>(b)); }
std::int32_t Controller::get_digital_new_press(controller_digital_e_t b) {
    static bool last[32] = {};
    const bool v = sim::controllerDigital(static_cast<int>(b)) != 0;
    const bool edge = v && !last[b & 31];
    last[b & 31] = v;
    return edge;
}
std::int32_t Controller::rumble(const char* p) { sim::logRumble(p); return 1; }
}  // namespace v5

namespace battery {
double get_capacity(void) { return 100.0; }
}
namespace adi {
std::int32_t DigitalIn::get_new_press() const { return 0; }
std::int32_t Encoder::get_value() const { return 0; }
std::int32_t Encoder::reset() const { return 1; }
}  // namespace adi
}  // namespace pros
