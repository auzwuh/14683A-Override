// 15 s match autonomous, one routine for all four starts.
//
// Rules it is built on (Game Manual v2.1):
//  SG1   start touching the Field Perimeter on our side, NOT touching a Toggle, Goal, Loader or
//        Load Zone, one robot per Quadrant.
//  SG7   stay on our side of the Autonomous Line.
//  SG11  no Match Loads in autonomous, so the Loaders are no use here.
//  SC5   a yellow Pin half only scores if the Toggle of its Quadrant shows our colour when the
//        15 s end, and the robot is not touching that Toggle (SC4). The Toggle is worth every
//        yellow point in the Quadrant, including the Pin that starts in the neutral Short Goal.
//  SC8   Win Point needs both robots off the wall at the end.
//
// The routine: preload (YELLOW HALF UP) onto our Alliance Goal, the corner Cup + yellow Pin on top
// of it, then the Toggle, then back off the wall. 30 points with the Toggle ours (two Pins on the
// Alliance Goal and the Short Goal's own Pin), 0 without it. The Toggle is done by 12 s.
// Tried and dropped (simulated, 24 Nov): a third object, 40 points but the Toggle then ends at 13.6 s
// on average and 15.0 at worst. The Toggle before the Cup, 20 points safe by 9 s but the Cup is then
// 39 in from the last wall fix and gets tipped in 15 of 96 runs (26 of 32 on an uncalibrated robot).
//
// It is written once, for the red robot on the red wall (Quadrant Red 1), in field inches (centre
// origin, +y toward the red wall, heading 0 = facing the red wall). Red 2 is the same routine
// mirrored in the red diagonal, the two blue ones are those two turned half round.
#include <cassert>
#include "main.h"
#include "gen/chassis/chassis.hpp"
#include "skills_support.hpp"
#include "autons.hpp"

extern arc::Chassis chassis;

namespace {

struct Pt { float x, y; };
bool mirrored = false, blue = false;

Pt at(float x, float y) {
    if (mirrored) { const float t = x; x = -y; y = -t; }
    if (blue) { x = -x; y = -y; }
    return {x, y};
}
float hdg(float h) {
    if (mirrored) h = 270.0f - h;
    if (blue) h += 180.0f;
    return std::fmod(h + 720.0f, 360.0f);
}

// The mirror has to land on the other red Goal and the half turn on the blue ones
[[maybe_unused]] const bool transformChecked = [] {
    auto same = [](Pt p, float x, float y) { return std::fabs(p.x - x) < 0.01f && std::fabs(p.y - y) < 0.01f; };
    mirrored = true;
    assert(same(at(-23.40f, 46.80f), -46.80f, 23.40f) && hdg(180.0f) == 90.0f);    // Red 2: Goal (48,24), back on the audience wall
    blue = true;
    assert(same(at(-23.40f, 46.80f), 46.80f, -23.40f) && hdg(180.0f) == 270.0f);   // Blue 2: Goal (96,120), back on the far wall
    mirrored = false;
    assert(same(at(-23.40f, 46.80f), 23.40f, -46.80f) && hdg(180.0f) == 0.0f);     // Blue 1: Goal (120,96), back on the blue wall
    blue = false;
    return true;
}();

// ---- Red 1 layout. TUNE on the field, the other three starts follow.
// Start: back on the wall in the 18.2 in slot between the Load Zone tape and the wall stack Cups
// (2.3 in spare each side), 32.8 in from the corner. The folded lift is what touches the wall.
constexpr float kStartX = -37.40f, kStartY = 59.94f;
constexpr float kPullY = 56.95f;                      // off the wall far enough to turn with the lift low
constexpr float kGoalX = -23.40f, kGoalY = 46.80f;    // our Alliance Goal
constexpr float kLaneY = 58.50f;                      // lane between the Goal and the wall stack, 1.7 in spare each side
constexpr float kLaneInX = -38.50f;                   // where it turns onto the lane, front corners 4 in short of the Cups
constexpr float kToggleX = -3.00f;                    // where the robot stands at the Toggle. The paddle is 6.4 in to its right,
                                                      // and this keeps it 2.5 in inside the Toggle's end on all four starts
constexpr float kPreGapY = 60.20f;                    // front rails 3.5 in off the wall, toggleVisit() closes the rest
constexpr float kOffWallY = 55.00f;                   // clear of the wall and the Toggle when the 15 s end
constexpr float kCupX = -46.80f, kCupY = 46.80f;      // corner Cup + yellow Pin (24,24), on the line between our two Quadrants
constexpr float kWallCupX = -23.40f, kWallCupY = 68.66f;   // wall stack behind the Alliance Goal: the middle Cup, with the yellow Pin
constexpr float kWallBackOffIn = 5.5f;                // back off the Goal this far, then turn at the wall stack. Further back the robot comes
                                                      // in flatter to the wall (36 deg off square from here) and its front corner meets the wall sooner
constexpr float kWallStageIn = 11.5f;                 // fast up to here from the Cup, then the creep
constexpr float kWallOutX = -31.45f, kWallOutY = 57.65f;   // back out of the wall stack to here, on the line it came in on and on the
                                                      // wall lane. About 1 in spare to the Cups in the turn and to the Goal base on the lane: at y 58.0 the
                                                      // front corner brushed the Cups, at 57.3 the side passed the Goal base by 0.2 in (simulated)
constexpr float kWallOffY = 50.00f;                   // back off the Toggle to here before turning at the Goal with the wall Cup + Pin
constexpr float kBackOffIn = 8.0f;                    // straight back off the Goal before steering (its base is between the rails)

void run(bool mirror, bool isBlue, bool wallStack = false) {
    mirrored = mirror;
    blue = isBlue;
    const Pt start = at(kStartX, kStartY), pull = at(kStartX, kPullY), goal = at(kGoalX, kGoalY), cup = at(kCupX, kCupY);
    const Pt wallCup = at(kWallCupX, kWallCupY), wallOut = at(kWallOutX, kWallOutY), wallOff = at(kToggleX, kWallOffY);
    const Pt laneIn = at(kLaneInX, kLaneY), lane = at(kToggleX, kLaneY), pre = at(kToggleX, kPreGapY), offWall = at(kToggleX, kOffWallY);
    const auto alliance = dr4b::kAllianceGoal;

    chassis.setPose(start.x, start.y, hdg(180.0f));
    skills::markStart();
    skills::resetGoals();
    toggle::motors().brake();          // hold the paddle where it was parked (long end back)
    localizer.beginSquare(true);       // leave with the heading of the wall it starts on
    pros::delay(250);
    localizer.endSquare(hdg(180.0f), true);
    localizer.startContinuous();

    // 1. Preload onto the Alliance Goal
    chassis.moveToPoint(pull.x, pull.y, {.timeout = 471, .velocityExit = 20.0f, .errorExit = 1.5f});
    skills::liftForGoal(alliance, 1);
    chassis.turnToPoint(goal.x, goal.y, {.timeout = 692, .velocityExit = 10.0f, .errorExit = 1.5f, .settleTimeMs = 80});
    skills::reachTo(goal.x, goal.y, skills::goalStopIn(alliance, 1), 855);
    skills::aimAt(goal.x, goal.y);
    dr4b::waitSettled(700);
    skills::releaseOn(alliance, 1);
    pros::delay(120);

    chassis.moveToPoint(pull.x, pull.y, {.timeout = 855, .velocityExit = 20.0f, .errorExit = 1.5f, .forwards = false, .async = true});
    chassis.waitUntil(3.5f);
    dr4b::to(dr4b::kFloor);
    chassis.waitUntilDone();

    // 2. Corner Cup + Pin, 14 in from where the robot just squared on the wall: turn at it, creep in
    // with the rollers on, the claw sensor stops the creep
    chassis.turnToPoint(cup.x, cup.y, {.timeout = 900, .velocityExit = 10.0f, .errorExit = 1.5f, .settleTimeMs = 80});
    chassis.moveToPoint(cup.x, cup.y, {.timeout = 1300, .velocityExit = 1000.0f, .errorExit = 5.75f, .maxSpeed = 40.0f, .halfPlaneExit = 0, .async = true});
    chassis.waitUntil(0.0f);
    rclaw::intake();
    rclaw::stopDriveOnObject(1300);
    chassis.waitUntilDone();
    skills::noteGrip(rclaw::confirmGrip(400));
    if (skills::holding) {
        skills::liftForGoal(alliance, 2);
        chassis.turnToPoint(goal.x, goal.y, {.timeout = 1100, .velocityExit = 60.0f, .errorExit = 3.0f, .settleTimeMs = 0});
        skills::reachTo(goal.x, goal.y, skills::goalStopIn(alliance, 2), 950);
        skills::aimAt(goal.x, goal.y);
        dr4b::waitSettled(700);
        skills::releaseOn(alliance, 2);
        pros::delay(120);
        // straight back off the Goal, its base is between the rails
        const arc::Pose p = chassis.getPose();
        const float d = std::hypot(p.x - goal.x, p.y - goal.y), k = d > 1.0f ? (wallStack ? kWallBackOffIn : kBackOffIn) / d : 0.0f;
        chassis.moveToPoint(p.x + (p.x - goal.x) * k, p.y + (p.y - goal.y) * k,
                            {.timeout = 700, .velocityExit = 20.0f, .errorExit = 1.5f, .forwards = false, .async = true});
        chassis.waitUntil(2.5f);
        if (wallStack) dr4b::to(dr4b::kFloor);
        chassis.waitUntilDone();
    }

    // 2b. Wall stack (only on the "+ wall stack" rows). The three Cups touch, the claw wedges in on the
    // middle one and the claw sensor stops the creep. The robot comes in at an angle: square to the
    // wall it would have to turn between the stack and the Goal, and there is no room for that.
    // The claw has to reach past the front rails for this (it does on the robot, Austin 25 Nov).
    if (wallStack) {
        chassis.turnToPoint(wallCup.x, wallCup.y, {.timeout = 900, .velocityExit = 10.0f, .errorExit = 1.5f, .settleTimeMs = 80});
        {
            const arc::Pose p = chassis.getPose();
            const float d = std::hypot(p.x - wallCup.x, p.y - wallCup.y), k = d > kWallStageIn ? kWallStageIn / d : 1.0f;
            chassis.moveToPoint(wallCup.x + (p.x - wallCup.x) * k, wallCup.y + (p.y - wallCup.y) * k, {.timeout = 900, .velocityExit = 25.0f, .errorExit = 2.0f});
        }
        chassis.moveToPoint(wallCup.x, wallCup.y, {.timeout = 1072, .velocityExit = 1000.0f, .errorExit = 5.75f, .maxSpeed = 40.0f, .halfPlaneExit = 0, .async = true});
        chassis.waitUntil(0.0f);
        rclaw::intake();
        rclaw::stopDriveOnObject(1200);
        chassis.waitUntilDone();
        skills::noteGrip(rclaw::confirmGrip(400));
        // Back out onto the wall lane, lift up to the Toggle height on the way. The Cup + Pin stays in
        // the claw through the Toggle visit and is placed after it: the Toggle is worth 30 points, this
        // Cup + Pin 10, so the Toggle goes first and a late placement only loses its own 10.
        // (Placed before the Toggle, the Toggle was not finished at 15 s in 6 of 32 simulated runs.)
        chassis.moveToPoint(wallOut.x, wallOut.y, {.timeout = 800, .velocityExit = 20.0f, .errorExit = 1.5f, .forwards = false, .async = true});
        chassis.waitUntil(3.0f);
        dr4b::to(dr4b::kToggleUp);
        chassis.waitUntilDone();
        dr4b::waitSettled(500);   // next to the wall the lift is low or above 35 deg before the robot turns
    }
    const bool carried = wallStack && skills::holding;   // the wall Cup + Pin is in the claw

    // 3. Toggle. Lift up to kToggleUp first: next to a wall the lift is low or above 35 deg.
    // Stop and turn onto the lane. Rolled through as one curve the robot swung 2 in wide and its
    // side went through the wall stack Cups in every simulated run.
    dr4b::to(dr4b::kToggleUp);
    if (!wallStack) {
        chassis.turnToPoint(laneIn.x, laneIn.y, {.timeout = 800, .velocityExit = 60.0f, .errorExit = 3.0f, .settleTimeMs = 0});
        chassis.moveToPoint(laneIn.x, laneIn.y, {.timeout = 900, .velocityExit = 20.0f, .errorExit = 1.0f});
    }
    chassis.turnToPoint(lane.x, lane.y, {.timeout = 900, .velocityExit = 10.0f, .errorExit = 1.5f, .settleTimeMs = 80});
    chassis.moveToPoint(lane.x, lane.y, {.timeout = 1700, .velocityExit = 20.0f, .errorExit = 1.5f});
    chassis.turnToHeading(hdg(0.0f), {.timeout = 899, .velocityExit = 10.0f, .errorExit = 1.5f, .settleTimeMs = 80});
    chassis.moveToPoint(pre.x, pre.y, {.timeout = 280, .maxSpeed = 70.0f});
    skills::toggleVisit(6.50f, 1.08f);
    std::printf("[t %5.2f] Toggle done\n", skills::elapsedMs() / 1000.0);
    // Off the wall and off the Toggle: a Toggle a robot touches is not set (SC4), and the Win Point
    // needs the robot off the Field Perimeter (SC8)
    if (!carried) {
        chassis.moveToPoint(offWall.x, offWall.y, {.timeout = 700, .forwards = false});
    } else {
        // 4. The wall Cup + Pin onto the Alliance Goal, third level, from the Toggle side
        chassis.moveToPoint(wallOff.x, wallOff.y, {.timeout = 800, .velocityExit = 20.0f, .errorExit = 1.5f, .forwards = false, .async = true});
        chassis.waitUntil(4.0f);
        skills::liftForGoal(alliance, 2);
        chassis.waitUntilDone();
        chassis.turnToPoint(goal.x, goal.y, {.timeout = 1100, .velocityExit = 60.0f, .errorExit = 3.0f, .settleTimeMs = 0});
        skills::reachTo(goal.x, goal.y, skills::goalStopIn(alliance, 2) + 4.0f, 1000, false, true);
        skills::liftGate(700);
        skills::reachTo(goal.x, goal.y, skills::goalStopIn(alliance, 2), 700);
        skills::aimAt(goal.x, goal.y, 1.5);
        dr4b::waitSettled(700);
        skills::releaseOn(alliance, 2);
        pros::delay(120);
        const arc::Pose p = chassis.getPose();
        const float d = std::hypot(p.x - goal.x, p.y - goal.y), k = d > 1.0f ? kBackOffIn / d : 0.0f;
        chassis.moveToPoint(p.x + (p.x - goal.x) * k, p.y + (p.y - goal.y) * k, {.timeout = 700, .forwards = false});
    }
    rclaw::hold();
    std::printf("[t %5.2f] match auton done\n", skills::elapsedMs() / 1000.0);
}

}  // namespace

namespace Auton {
void matchRed1() { run(false, false); }    // red, back on the red wall
void matchRed2() { run(true, false); }     // red, back on the audience wall
void matchBlue1() { run(false, true); }    // blue, back on the blue wall
void matchBlue2() { run(true, true); }     // blue, back on the far wall
// The same four, plus the wall stack behind the Alliance Goal: grabbed before the Toggle, placed after it. 40 points
void matchRed1Wall() { run(false, false, true); }
void matchRed2Wall() { run(true, false, true); }
void matchBlue1Wall() { run(false, true, true); }
void matchBlue2Wall() { run(true, true, true); }
}  // namespace Auton
