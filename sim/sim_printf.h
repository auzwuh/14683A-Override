// Force-included (-include) into every TU that sees robot headers, so robot
// printf() lines reach the sim log with a virtual timestamp and task name.
#pragma once
#include <cstdio>
extern "C" int sim_printf(const char* fmt, ...);
namespace std { using ::sim_printf; }
#define printf sim_printf
