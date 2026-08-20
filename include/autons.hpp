#pragma once

// Autonomous routines generated from ATTICUS_TERMINAL and adapted to the
// GenClient chassis API.  Register these in autonRoutines in main.cpp.
//
// All four are the SAME routine worked in a different Quadrant - set that
// Quadrant's Toggle, then put two Pins into its neutral Short Goal, where the
// Toggle makes the yellow halves count.  40 points each.
//
// <SG1>d forbids both robots of an Alliance starting in the same Quadrant, so
// these come in pairs: one robot runs the wall routine, the partner runs the
// other one.
//
//   red  : left  Quadrant (left wall)    + bottom Quadrant (bottom wall)
//   blue : right Quadrant (right wall)   + top    Quadrant (top wall)
//
// Red needs TWO Toggle rams and blue needs ONE: a Toggle cycles yellow -> blue
// -> red from the Field interior, so a second blue ram would push it to red and
// hand the Quadrant to the opponent.

namespace Auton {

// Red, starting flush against the LEFT perimeter at the Toggle.   ~11.8 s
void overrideRedLeft();

// Red, starting flush against the BOTTOM perimeter at the Toggle. ~11.8 s
void overrideRedBottom();

// Blue, starting flush against the RIGHT perimeter at the Toggle. ~10.8 s
void overrideBlueRight();

// Blue, starting flush against the TOP perimeter at the Toggle.   ~10.8 s
void overrideBlueTop();

}  // namespace Auton
