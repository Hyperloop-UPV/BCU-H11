#pragma once

#define INVERTER 1            // 1 = A / 2 = B / 3 = BOTH
#define DCLINK 1              // 1 = A / 2 = B / 3 = BOTH / 4 = Fixed by hand
#define TRIGONOMETRICS_CALC 1 // 1 => Trigonometrics from std / 2 = Cordic / 3 = LUT
#define SPEETEC 1             // 1 = Use Speetec 1 / 2 = Use Speetec 2 / 3 = Mix

#define USE_MATLAB_FOC_SPEED 1        // 1 = Use MATLAB FOC / 0 = Use custom FOC
#define USE_MATLAB_FOC_CURRENT 1      // 1 = Use MATLAB FOC / 0 = Use custom FOC
// Check the ethernet PHY and define the board version accordingly.

#if defined(USE_PHY_KSZ8041)
#define H11
#elif defined(USE_PHY_LAN8742) || defined(USE_PHY_LAN8700)
#define H10
#else
#error                                                                                             \
    "BCU Version not defined. Please define either USE_BCU_H10 or USE_BCU_H11 (or the corresponding ethernet PHY)"
#endif
