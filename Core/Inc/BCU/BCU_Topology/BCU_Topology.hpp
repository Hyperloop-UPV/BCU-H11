#pragma once
#include "ST-LIB.hpp"
#include "BCU/Data/Data.hpp"
#include "BCU/Wrappers/Wrappers.hpp"
#include "BCU/ThreePhaseMotor/ThreePhaseMotor.hpp"

namespace BCU {
    namespace Topology {
        //PWM Objects
        inline constexpr auto kMotorPhaseR_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_R_P,
           .channel = Pinout::kpwm_channel_R_P,
        };

        inline constexpr auto kMotorPhaseR_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_R_N,
           .channel = Pinout::kpwm_channel_R_N,
        };
        inline constexpr auto kMotorPhaseS_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_S_P,
           .channel = Pinout::kpwm_channel_S_P,
        };

        inline constexpr auto kMotorPhaseS_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_S_N,
           .channel = Pinout::kpwm_channel_S_N,
        };

        inline constexpr auto kMotorPhaseT_P = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_T_P,
           .channel = Pinout::kpwm_channel_T_P,
        };

        inline constexpr auto kMotorPhaseT_N = ST_LIB::TimerPin{
           .af = ST_LIB::TimerAF::PWM,
           .pin = Pinout::kPWM_T_N,
           .channel = Pinout::kpwm_channel_T_N,
        };
        //timer
        inline constexpr auto kmotor_timer = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_PWM},
             kMotorPhaseR_P, kMotorPhaseR_N, kMotorPhaseS_P, kMotorPhaseS_N, kMotorPhaseT_P, kMotorPhaseT_N);

        // PPU signals
        inline constexpr auto kPPUReady_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_B);
        inline constexpr auto kPPUFault_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_B);
        inline constexpr auto kPPUReady_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_A);
        inline constexpr auto kPPUFault_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_A);
        inline constexpr auto kBufferEnable = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kBuffer_Enable);

        // LEDs
        inline constexpr auto kLED_Operational = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Operational);
        inline constexpr auto kLED_Fault = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Fault);
        inline constexpr auto kLED_Connecting = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Connecting);
        inline constexpr auto kLED_Can = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Can);
        inline constexpr auto kLED_Accelerating = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Accelerating);
        inline constexpr auto kLED_Braking = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Braking);

        // Hall sensor supply
        inline constexpr auto kHall_Supply_A = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_A);
        inline constexpr auto kHall_Supply_B = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_B);

        //Hall Sensors
        inline constexpr auto kCurrentSenseR_A = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_R_A,ST_LIB::ADCDomain::Resolution::BITS_16);
        inline constexpr auto kCurrentSenseR_B = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_R_B,ST_LIB::ADCDomain::Resolution::BITS_16);
        inline constexpr auto kCurrentSenseS_A = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_S_A,ST_LIB::ADCDomain::Resolution::BITS_16);
        inline constexpr auto kCurrentSenseS_B = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_S_B,ST_LIB::ADCDomain::Resolution::BITS_16);
        inline constexpr auto kCurrentSenseT_A = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_T_A,ST_LIB::ADCDomain::Resolution::BITS_16);
        inline constexpr auto kCurrentSenseT_B = ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_T_B,ST_LIB::ADCDomain::Resolution::BITS_16);

        // Speedtec
        inline constexpr auto kSpeetecSupply = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kSpeedtecSupply);

        inline constexpr auto kSpeedtec1_Input1 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec1_IN1,
            .channel = Pinout::kSpeedtec1_Channel_IN1,
        };
        inline constexpr auto kSpeedtec1_Input2 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec1_IN2,
            .channel = Pinout::kSpeedtec1_Channel_IN2
        };
        inline constexpr auto kSpeedtec2_Input1 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec2_IN1,
            .channel = Pinout::kSpeedtec2_Channel_IN1
        };
        inline constexpr auto kSpeedtec2_Input2 = ST_LIB::TimerPin{
            .af = ST_LIB::TimerAF::Encoder,
            .pin = Pinout::kSpeedtec2_IN2,
            .channel = Pinout::kSpeedtec2_Channel_IN2
        };

        inline constexpr auto ktimer_speetec1 = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_speetec1},
             kSpeedtec1_Input1, kSpeedtec1_Input2);
        
             inline constexpr auto ktimer_speetec2 = ST_LIB::TimerDomain::Timer(
            {.request = Pinout::ktimer_speetec2},
             kSpeedtec2_Input1, kSpeedtec2_Input2); 
        // Temperature sensors
        inline constexpr auto kTempSensorA = ST_LIB::ADCDomain::ADC(Pinout::kTempSensorA);
        inline constexpr auto kTempSensorB = ST_LIB::ADCDomain::ADC(Pinout::kTempSensorB);

        // Voltage sensors
        inline constexpr auto kVoltageSensorA = ST_LIB::ADCDomain::ADC(Pinout::kVoltageSensorA);
        inline constexpr auto kVoltageSensorB = ST_LIB::ADCDomain::ADC(Pinout::kVoltageSensorB);

        // Ethernet (conditional)
        #ifdef STLIB_ETH
        #if defined(USE_PHY_LAN8742)
        inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
            ST_LIB::EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #elif defined(USE_PHY_LAN8700)
        #if MASK_TO_24 == 1
        inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
            ST_LIB::EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.255.0"
        );
        #else
        inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
            ST_LIB::EthernetDomain::PINSET_H10,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #endif
        #elif defined(USE_PHY_KSZ8041)
        #if MASK_TO_24 == 1
        inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
            ST_LIB::EthernetDomain::PINSET_H11,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.255.0"
        );
        #else
        inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
            ST_LIB::EthernetDomain::PINSET_H11,
            "05:80:e8:55:61:09",
            "192.168.1.5",
            "255.255.0.0"
        );
        #endif
        #else
        #error "Ethernet PHY not defined"
        #endif
        #endif
    } // namespace BCU_Topology
    using Board = ST_LIB::Board< 
    #ifdef STLIB_ETH
        Topology::eth,
    #endif
        Topology::kPPUReady_B, Topology::kPPUFault_B, Topology::kPPUReady_A, Topology::kPPUFault_A, Topology::kBufferEnable,
        Topology::kLED_Operational, Topology::kLED_Fault, Topology::kLED_Connecting, Topology::kLED_Can, Topology::kLED_Accelerating, Topology::kLED_Braking,
        Topology::kHall_Supply_A, Topology::kHall_Supply_B,
        Topology::kCurrentSenseR_A, Topology::kCurrentSenseR_B, Topology::kCurrentSenseS_A, Topology::kCurrentSenseS_B, Topology::kCurrentSenseT_A, Topology::kCurrentSenseT_B,
        Topology::kSpeetecSupply,
        Topology::kTempSensorA, Topology::kTempSensorB,
        Topology::kVoltageSensorA, Topology::kVoltageSensorB,
        Topology::kmotor_timer,Topology::ktimer_speetec1, Topology::ktimer_speetec2
        >;
    namespace Types{
        //Motor 
        using MotorTimer = ST_LIB::TimerWrapper<Topology::kmotor_timer>;
        using PhaseR = decltype(std::declval<MotorTimer>().template get_dual_pwm<Topology::kMotorPhaseR_P, Topology::kMotorPhaseR_N>());
        using PhaseS = decltype(std::declval<MotorTimer>().template get_dual_pwm<Topology::kMotorPhaseS_P, Topology::kMotorPhaseS_N>());
        using PhaseT = decltype(std::declval<MotorTimer>().template get_dual_pwm<Topology::kMotorPhaseT_P, Topology::kMotorPhaseT_N>());
        using BufferEnable = decltype(DigitalOutputWrapper<Board, Types::OLogic::N_CLOSE, Topology::kBufferEnable>());
        
        using SynchronousMotor = Devices::ThreePhaseMotor<BCU::Board, BCU::Types::MotorTimer, BCU::Types::PhaseR, BCU::Types::PhaseS, BCU::Types::PhaseT, BCU::Types::BufferEnable>;
        //Current Sensor
        using HallSupply_A = decltype(DigitalOutputWrapper<Board, Types::OLogic::N_OPEN, Topology::kHall_Supply_A>());
        using HallSupply_B = decltype(DigitalOutputWrapper<Board, Types::OLogic::N_OPEN, Topology::kHall_Supply_B>());

        using CurrentSenseA = Sensor<Board,Types::CurrentSense_Data,Types::HallSupply_A,Configuration::currentSense_A,
                                Topology::kCurrentSenseR_A,BCU::Topology::kCurrentSenseS_A,Topology::kCurrentSenseT_A>;
        using CurrentSenseB = Sensor<Board,Types::CurrentSense_Data,Types::HallSupply_B,Configuration::currentSense_B,
                                Topology::kCurrentSenseR_B,Topology::kCurrentSenseS_B,Topology::kCurrentSenseT_B>;

        //Voltage Sensors
        using VoltageSense = Sensor<Board,Types::VoltageSense_Data,Devices::NoSupply,Configuration::voltageSense,
                                Topology::kVoltageSensorA,Topology::kVoltageSensorB>;

        //Temperature Sensor
        using TempSense = Sensor<Board,Types::TempSense_Data,Devices::NoSupply,Configuration::tempSense,
                                Topology::kTempSensorA,Topology::kTempSensorB>;

        //Encoder 
        using SpeetecSupply = decltype(DigitalOutputWrapper<Board, Types::OLogic::N_OPEN, Topology::kSpeetecSupply>());
        using EncoderTimer1 = ST_LIB::TimerWrapper<Topology::ktimer_speetec1>;
        using EncoderTimer2 = ST_LIB::TimerWrapper<Topology::ktimer_speetec2>;

    }
    

} //namespace BCU
