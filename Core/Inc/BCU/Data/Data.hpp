#pragma once
#include "BCU/Config/Config.hpp"
#include "BCU/Sensor/Sensor.hpp"
#include "ST-LIB.hpp"


namespace BCU {

namespace Configuration{
    inline constexpr int deadTime_ns = 300;
    inline constexpr int switchingFrequency_Hz = 1000;

    inline constexpr SensorConfig::Config currentSense_R_A{1.0f, 0.0f};
    inline constexpr SensorConfig::Config currentSense_S_A{1.0f, 0.0f};
    inline constexpr SensorConfig::Config currentSense_T_A{1.0f, 0.0f};
    inline constexpr std::array<SensorConfig::Config,3> currentSense_A{currentSense_R_A,currentSense_S_A,currentSense_T_A};


    inline constexpr SensorConfig::Config currentSense_R_B{1.0f, 0.0f};
    inline constexpr SensorConfig::Config currentSense_S_B{1.0f, 0.0f};
    inline constexpr SensorConfig::Config currentSense_T_B{1.0f, 0.0f};
    inline constexpr std::array<SensorConfig::Config,3> currentSense_B{currentSense_R_B,currentSense_S_B,currentSense_T_B};

    inline constexpr SensorConfig::Config voltageSense_A{1.0f, 0.0f};
    inline constexpr SensorConfig::Config voltageSense_B{1.0f, 0.0f};

    inline constexpr SensorConfig::Config tempSense_A{1.0f, 0.0f};
    inline constexpr SensorConfig::Config tempSense_B{1.0f, 0.0f};
}
namespace Pinout {
//PWM R
inline constexpr auto& kPWM_R_P = ST_LIB::PE9;
inline constexpr auto& kPWM_R_N = ST_LIB::PE8;

inline constexpr auto kpwm_channel_R_P = ST_LIB::TimerChannel::CHANNEL_1;
inline constexpr auto kpwm_channel_R_N = ST_LIB::TimerChannel::CHANNEL_1_NEGATED;
//PWM S
inline constexpr auto& kPWM_S_P = ST_LIB::PE11;
inline constexpr auto& kPWM_S_N = ST_LIB::PE10;

inline constexpr auto kpwm_channel_S_P = ST_LIB::TimerChannel::CHANNEL_2;
inline constexpr auto kpwm_channel_S_N = ST_LIB::TimerChannel::CHANNEL_2_NEGATED;
//PWM T
inline constexpr auto& kPWM_T_P = ST_LIB::PE13;
inline constexpr auto& kPWM_T_N = ST_LIB::PE12;

inline constexpr auto kpwm_channel_T_P = ST_LIB::TimerChannel::CHANNEL_3;
inline constexpr auto kpwm_channel_T_N = ST_LIB::TimerChannel::CHANNEL_3_NEGATED;
// timer
inline constexpr auto ktimer_PWM = ST_LIB::TimerRequest::Advanced_1;

// PPU signals
// inputs
inline constexpr auto& kREADY_B = ST_LIB::PE14;
inline constexpr auto& kFAULT_B = ST_LIB::PE15;
inline constexpr auto& kREADY_A = ST_LIB::PB5;
inline constexpr auto& kFAULT_A = ST_LIB::PB6;
// outputs
inline constexpr auto& kBuffer_Enable = ST_LIB::PF4;

// Current sense signals
inline constexpr auto& kCurrentSense_R_A = ST_LIB::PA0;
inline constexpr auto& kCurrentSense_R_B = ST_LIB::PA6;
inline constexpr auto& kCurrentSense_S_A = ST_LIB::PA4;
inline constexpr auto& kCurrentSense_S_B = ST_LIB::PB0;
inline constexpr auto& kCurrentSense_T_A = ST_LIB::PA5;
inline constexpr auto& kCurrentSense_T_B = ST_LIB::PB1;

inline constexpr auto& kHall_Supply_A = ST_LIB::PD12;
inline constexpr auto& kHall_Supply_B = ST_LIB::PD11;

// Leds
inline constexpr auto& kLED_Operational = ST_LIB::PG13;
inline constexpr auto& kLED_Fault = ST_LIB::PG12;
inline constexpr auto& kLED_Connecting = ST_LIB::PG11;
inline constexpr auto& kLED_Can = ST_LIB::PG10;
inline constexpr auto& kLED_Accelerating = ST_LIB::PG9;
inline constexpr auto& kLED_Braking = ST_LIB::PG6;

//speetec 1
inline constexpr auto& kSpeedtec1_IN1 = ST_LIB::PF0;
inline constexpr auto& kSpeedtec1_IN2 = ST_LIB::PF1;

inline constexpr auto kSpeedtec1_Channel_IN1 = ST_LIB::TimerChannel::CHANNEL_1;
inline constexpr auto kSpeedtec1_Channel_IN2 = ST_LIB::TimerChannel::CHANNEL_2;
inline constexpr auto ktimer_speetec1 = ST_LIB::TimerRequest::GeneralPurpose32bit_23;

//Speetec 2
inline constexpr auto& kSpeedtec2_IN1 = ST_LIB::PE5;
inline constexpr auto& kSpeedtec2_IN2 = ST_LIB::PE6;

inline constexpr auto kSpeedtec2_Channel_IN1= ST_LIB::TimerChannel::CHANNEL_1;
inline constexpr auto kSpeedtec2_Channel_IN2 = ST_LIB::TimerChannel::CHANNEL_2;
inline constexpr auto ktimer_speetec2 = ST_LIB::TimerRequest::GeneralPurpose_15;


//Speedtec supply
inline constexpr auto& kSpeedtecSupply = ST_LIB::PD10;

// temp sensors
inline constexpr auto& kTempSensorA = ST_LIB::PF7;
inline constexpr auto& kTempSensorB = ST_LIB::PF8;

// Voltage sensors
inline constexpr auto& kVoltageSensorA = ST_LIB::PF3;
inline constexpr auto& kVoltageSensorB = ST_LIB::PF5;

} // namespace Pinout

namespace Types {
      //OUTPUT LOGIC (Normally Open or Normally Closed)
        enum OLogic{
            N_OPEN,
            N_CLOSE,
        };

    

    struct CurrentSense_Data{
        union{
        struct{
            float current_R;
            float current_S;
            float current_T;
        };
        float sensor_data[3];
        };
        
    };
    struct VoltageSense_Data{
        union{
        struct{
            float voltage_A;
            float voltage_B;
        };
        float sensor_data[2];
    };
        
    };
    struct TempSense_Data{
        union{
        struct{
        float temp_A;
        float temp_B; 
        };
        float sensor_data[2]; 
        }; 
    };
    
}

} // namespace BCU
