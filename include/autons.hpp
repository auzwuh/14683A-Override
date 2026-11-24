#pragma once

namespace Auton {

void ramseteLqrTestRoutine();
void ramseteLqrTauTest();

// 15 s match autons (src/match_autons.cpp). 1 = back on our Alliance Station wall, 2 = back on the
// wall to the driver's right (red: audience wall, blue: far wall).
void matchRed1();
void matchRed2();
void matchBlue1();
void matchBlue2();

}
