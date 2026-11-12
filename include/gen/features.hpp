#pragma once

#define ARC_RAMSETE_LQR_ENABLED 0

// The field routines still target the retired winch-style mechanism constants.
// Keep them out of debug builds until the measured cascade/rotator values are
// available and the routines are rewritten around the current mechanism.
#ifndef ARC_FIELD_AUTONS_ENABLED
#define ARC_FIELD_AUTONS_ENABLED 0
#endif
