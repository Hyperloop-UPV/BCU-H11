#pragma once
#include "ST-LIB.hpp"
#include "BCU/Data/Data.hpp"


namespace BCU {
    namespace BCU_Topology {
        //PWM Objects
        constexpr auto kMotorPhaseR_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_R_P,
           .channel = Pinout::kpwm_channel_R_P,
        };

        constexpr auto kMotorPhaseR_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_R_N,
           .channel = Pinout::kpwm_channel_R_N,
        };
        constexpr auto kMotorPhaseS_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_S_P,
           .channel = Pinout::kpwm_channel_S_P,
        };

        constexpr auto kMotorPhaseS_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_S_N,
           .channel = Pinout::kpwm_channel_S_N,
        };

        constexpr auto kMotorPhaseT_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_T_P,
           .channel = Pinout::kpwm_channel_T_P,
        };

        constexpr auto kMotorPhaseT_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_T_N,
           .channel = Pinout::kpwm_channel_T_N,
        };
        //timer
        constexpr auto kmotor_timer = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_PWM},
             kMotorPhaseR_P, kMotorPhaseR_N, kMotorPhaseS_P, kMotorPhaseS_N, kMotorPhaseT_P, kMotorPhaseT_N);

        // PPU signals
        constexpr auto kPPUReady_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_B);
        constexpr auto kPPUFault_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_B);
        constexpr auto kPPUReady_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_A);
        constexpr auto kPPUFault_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_A);
        constexpr auto kBuffer_Enable = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kBuffer_Enable);

        // LEDs
        constexpr auto kLED_Operational = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Operational);
        constexpr auto kLED_Fault = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Fault);
        constexpr auto kLED_Connecting = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Connecting);
        constexpr auto kLED_Can = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Can);
        constexpr auto kLED_Accelerating = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Accelerating);
        constexpr auto kLED_Braking = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Braking);

        // Hall sensor supply
        constexpr auto kHall_Supply_A = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_A);
        constexpr auto kHall_Supply_B = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_B);

        //Hall Sensors
        constexpr auto kCurrentSenseR_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_R_A);
        constexpr auto kCurrentSenseR_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_R_B);
        constexpr auto kCurrentSenseS_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_S_A);
        constexpr auto kCurrentSenseS_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_S_B);
        constexpr auto kCurrentSenseT_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_T_A);
        constexpr auto kCurrentSenseT_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kCurrentSense_T_B);

        // Speedtec
        constexpr auto kSpeedtecSupply = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kSpeedtecSupply);

        constexpr auto kSpeedtec1_Input1 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec1_IN1,
            .channel = Pinout::kSpeedtec1_Channel_IN1,
        };
        constexpr auto kSpeedtec1_Input2 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec1_IN2,
            .channel = Pinout::kSpeedtec1_Channel_IN2
        };
        constexpr auto kSpeedtec2_Input1 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec2_IN1,
            .channel = Pinout::kSpeedtec2_Channel_IN1
        };
        constexpr auto kSpeedtec2_Input2 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec2_IN2,
            .channel = Pinout::kSpeedtec2_Channel_IN2
        };

        constexpr auto ktimer_speetec1 = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_speetec1},
             kSpeedtec1_Input1, kSpeedtec1_Input2); //Shared timer between speetec 1 and 3
        
             constexpr auto ktimer_speetec2 = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_speetec2},
             kSpeedtec2_Input1, kSpeedtec2_Input2); 
        // Temperature sensors
        constexpr auto kTempSensorA = ST_LIB::ADCDomain::ADC(Pinout::kTempSensorA);
        constexpr auto kTempSensorB = ST_LIB::ADCDomain::ADC(Pinout::kTempSensorB);

        // Voltage sensors
        constexpr auto kVoltageSensorA = ST_LIB::ADCDomain::ADC(Pinout::kVoltageSensorA);
        constexpr auto kVoltageSensorB = ST_LIB::ADCDomain::ADC(Pinout::kVoltageSensorB);

        // Ethernet (conditional)
        #ifdef STLIB_ETH
        #if defined(USE_PHY_LAN8742)
        constexpr auto eth = EthernetDomain::Ethernet(
            EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #elif defined(USE_PHY_LAN8700)
        #if MASK_TO_24 == 1
        constexpr auto eth = EthernetDomain::Ethernet(
            EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.255.0"
        );
        #else
        constexpr auto eth = EthernetDomain::Ethernet(
            EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #endif
        #elif defined(USE_PHY_KSZ8041)
        #if MASK_TO_24 == 1
        constexpr auto eth = EthernetDomain::Ethernet(
            EthernetDomain::PINSET_H11,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.255.0"
        );
        #else
        constexpr auto eth = EthernetDomain::Ethernet(
            EthernetDomain::PINSET_H11,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #endif
        #else
        #error "Ethernet PHY not defined"
        #endif
        #endif

        // Main board register
    using Board = ST_LIB::Board<kMotorPhaseR_P, kMotorPhaseR_N, kMotorPhaseS_P, kMotorPhaseS_N, kMotorPhaseT_P, kMotorPhaseT_N,
        kPPUReady_B, kPPUFault_B, kPPUReady_A, kPPUFault_A, kBuffer_Enable,
        kLED_Operational, kLED_Fault, kLED_Connecting, kLED_Can, kLED_Accelerating, kLED_Braking,
        kHall_Supply_A, kHall_Supply_B,
        kCurrentSenseR_A, kCurrentSenseR_B, kCurrentSenseS_A, kCurrentSenseS_B, kCurrentSenseT_A, kCurrentSenseT_B,
        kSpeedtecSupply,
        kSpeedtec1_Input1, kSpeedtec1_Input2,
        kSpeedtec2_Input1, kSpeedtec2_Input2,
        kTempSensorA, kTempSensorB,
        kVoltageSensorA, kVoltageSensorB,
        kmotor_timer,ktimer_speetec1, ktimer_speetec2
        #ifdef STLIB_ETH
        , eth
        #endif
        >;
    } // namespace BCU_Topology

} //namespace BCU
