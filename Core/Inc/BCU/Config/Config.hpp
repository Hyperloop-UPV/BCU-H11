#pragma once

#define H11
#define INVERTER 1            // 1 = A / 2 = B / 3 = BOTH
#define DCLINK 1              // 1 = A / 2 = B / 3 = BOTH
#define TRIGONOMETRICS_CALC 1 // 1 => Trigonometrics from std / 2 = Cordic / 3 = LUT
#define SPEETEC 1             // 1 = Use Speetec 1 / 2 = Use Speetec 2 / 3 = Mix

#if defined(H11) && !defined(USE_PHY_KSZ8041)
    Compile_error("The Board H11 uses KSZ8041 as ethernet");
#endif

#if defined(H10) && !defined(USE_PHY_LAN8700)
    Compile_error("The Board H10 uses LAN8700 as ethernet")
#endif
