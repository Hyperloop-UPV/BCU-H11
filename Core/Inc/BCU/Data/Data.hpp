#pragma once 
#include "Config/Config.hpp"
#include "ST-LIB.hpp"

nampespace Pinout {
    //PPUs signals
    inline constexpr Pin& kPWM_R = Pin::PE9;
    inline constexpr Pin& kPWM_R_N = Pin::PE8;
    inline constexpr Pin& kPWM_S = Pin::PE11;
    inline constexpr Pin& kPWM_S_N = Pin::PE10;
    inline constexpr Pin& kPWM_T = Pin::PE13;
    inline constexpr Pin& kPWM_T_N = Pin::PE12;
    
    inline constexpr Pin& kREADY_B = Pin::PE14;
    inline constexpr Pin& kFAULT_B = Pin::PE15;
    inline constexpr Pin& kREADY_A = Pin::PB5;
    inline constexpr Pin& kFAULT_A = Pin::PB6;
    
    inline constexpr Pin& Buffer_Enable = Pin::PF4;
    //Current sense signals
    inline constexpr Pin& kCurrentSense_R_A = Pin::PA0;
    inline constexpr Pin& kCurrentSense_R_B = Pin::PA6;
    inline constexpr Pin& kCurrentSense_S_A = Pin::PA4;
    inline constexpr Pin& kCurrentSense_S_B = Pin::PB0;
    inline constexpr Pin& kCurrentSense_T_A = Pin::PA5;
    inline constexpr Pin& kCurrentSense_T_B = Pin::PB1;
    
    inline constexpr Pin& kHall_Supply_A = Pin::PD12;
    inline constexpr Pin& kHall_Supply_B = Pin::PD11;
    
    //Leds
    inline constexpr Pin& kLED_Operational = Pin::PG13;
    inline constexpr Pin& kLED_Fault = Pin::PG12;
    inline constexpr Pin& kLED_Connecting = Pin::PG11;
    inline constexpr Pin& kLED_Can = Pin::PG10;
    inline constexpr Pin& kLED_Accelerating = Pin::PG9;
    inline constexpr Pin& kLED_Braking = Pin::PG6;

    //Speetecs 
    inline constexpr Pin& kSpeedtec1_P = Pin::PF0;
    inline constexpr Pin& kSpeedtec1_N = Pin::PF1;
    inline constexpr Pin& kSpeedtec2_P = Pin::PF2;
    inline constexpr Pin& kSpeedtec2_N = Pin::PF9;
    inline constexpr Pin& kSpeedtec3_P = Pin::PE5;
    inline constexpr Pin& kSpeedtec3_N = Pin::PE6;

    inline constexpr Pin& kSpeedtecSupply = Pin::PD10; 

    //temp sensors
    inline constexpr Pin& kTempSensorA = Pin::PF7;
    inline constexpr Pin& kTempSensorB = Pin::PF8;

    //Voltage sensors
    inline constexpr Pin& kVoltageSensorA = Pin::PF3;
    inline constexpr Pin& kVoltageSensorB = Pin::PF5;

}

