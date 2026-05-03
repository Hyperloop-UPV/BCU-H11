#pragma once
#include "BCU/Data/Data.hpp"
#include "BCU/StateMachine/StateMachine.hpp"
namespace BCU {
namespace Topology {
// PWM Objects
inline constexpr auto kMotorPhaseU_P = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_U_P,
    .channel = Pinout::kpwm_channel_U_P,
};

inline constexpr auto kMotorPhaseU_N = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_U_N,
    .channel = Pinout::kpwm_channel_U_N,
};
inline constexpr auto kMotorPhaseV_P = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_V_P,
    .channel = Pinout::kpwm_channel_V_P,
};

inline constexpr auto kMotorPhaseV_N = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_V_N,
    .channel = Pinout::kpwm_channel_V_N,
};

inline constexpr auto kMotorPhaseW_P = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_W_P,
    .channel = Pinout::kpwm_channel_W_P,
};

inline constexpr auto kMotorPhaseW_N = ST_LIB::TimerPin{
    .af = ST_LIB::TimerAF::PWM,
    .pin = Pinout::kPWM_W_N,
    .channel = Pinout::kpwm_channel_W_N,
};
// timer
inline constexpr auto kmotor_timer = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::ktimer_PWM},
    kMotorPhaseU_P,
    kMotorPhaseU_N,
    kMotorPhaseV_P,
    kMotorPhaseV_N,
    kMotorPhaseW_P,
    kMotorPhaseW_N
);

// PPU signals
inline constexpr auto kPPUReady_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_B);
inline constexpr auto kPPUFault_B = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_B);
inline constexpr auto kPPUReady_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kREADY_A);
inline constexpr auto kPPUFault_A = ST_LIB::DigitalInputDomain::DigitalInput(Pinout::kFAULT_A);
inline constexpr auto kBufferEnable =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kBuffer_Enable);

// LEDs
inline constexpr auto kLED_Operational =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Operational);
inline constexpr auto kLED_Fault = ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Fault);
inline constexpr auto kLED_Connecting =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_Connecting);
inline constexpr auto kLED_SpaceVector =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_SpaceVector);
inline constexpr auto kLED_CurrentControl =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_CurrentControl);
inline constexpr auto kLED_SpeedControl =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kLED_SpeedControl);

#ifdef H11
// Hall sensor supply
inline constexpr auto kHall_Supply_A =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_A);
inline constexpr auto kHall_Supply_B =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kHall_Supply_B);
#endif
// Hall Sensors
inline constexpr auto kCurrentSenseU_A =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_U_A, ST_LIB::ADCDomain::Resolution::BITS_16);
inline constexpr auto kCurrentSenseU_B =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_U_B, ST_LIB::ADCDomain::Resolution::BITS_16);
inline constexpr auto kCurrentSenseV_A =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_V_A, ST_LIB::ADCDomain::Resolution::BITS_16);
inline constexpr auto kCurrentSenseV_B =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_V_B, ST_LIB::ADCDomain::Resolution::BITS_16);
inline constexpr auto kCurrentSenseW_A =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_W_A, ST_LIB::ADCDomain::Resolution::BITS_16);
inline constexpr auto kCurrentSenseW_B =
    ST_LIB::ADCDomain::ADC(Pinout::kCurrentSense_W_B, ST_LIB::ADCDomain::Resolution::BITS_16);

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

inline constexpr auto ktimer_speetec1 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::ktimer_speetec1},
    kSpeedtec1_Input1,
    kSpeedtec1_Input2
);

#ifdef H11
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

inline constexpr auto ktimer_speetec2 = ST_LIB::TimerDomain::Timer(
    {.request = Pinout::ktimer_speetec2},
    kSpeedtec2_Input1,
    kSpeedtec2_Input2
);
// Speetec supply
inline constexpr auto kSpeetecSupply =
    ST_LIB::DigitalOutputDomain::DigitalOutput(Pinout::kSpeedtecSupply);
#endif

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
    "05:80:e8:55:61:17",
    "192.168.2.17",
    "255.255.0.0"
);
#elif defined(USE_PHY_LAN8700)
#if MASK_TO_24 == 1
inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H10,
    "05:80:e8:55:61:17",
    "192.168.2.17",
    "255.255.0.0"
);
#else
inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H10,
    "05:80:e8:55:51:17",
    "192.168.2.17",
    "255.255.0.0"
);
#endif
#elif defined(USE_PHY_KSZ8041)
#if MASK_TO_24 == 1
inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H11,
    "05:80:e8:55:61:17",
    "192.168.2.17",
    "255.255.0.0"
);
#else
inline constexpr auto eth = ST_LIB::EthernetDomain::Ethernet(
    ST_LIB::EthernetDomain::PINSET_H11,
    "05:80:e8:55:61:17",
    "192.168.2.17",
    "255.255.0.0"
);
#endif
#else
#error "Ethernet PHY not defined"
#endif
#endif
} // namespace Topology

using BoardPolicy =
    ST_LIB::FaultPolicy<StateMachine::General_State_Machine, StateMachine::enter_Fault_state>;
using BCUBoard = ST_LIB::Board<
    BoardPolicy,
#ifdef STLIB_ETH
    Topology::eth,
#endif
    Topology::kPPUReady_B,
    Topology::kPPUFault_B,
    Topology::kPPUReady_A,
    Topology::kPPUFault_A,
    Topology::kBufferEnable,
    Topology::kLED_Operational,
    Topology::kLED_Fault,
    Topology::kLED_Connecting,
    Topology::kLED_SpaceVector,
    Topology::kLED_CurrentControl,
    Topology::kLED_SpeedControl,
    Topology::kCurrentSenseU_A,
    Topology::kCurrentSenseU_B,
    Topology::kCurrentSenseV_A,
    Topology::kCurrentSenseV_B,
    Topology::kCurrentSenseW_A,
    Topology::kCurrentSenseW_B,
    Topology::kTempSensorA,
    Topology::kTempSensorB,
    Topology::kVoltageSensorA,
    Topology::kVoltageSensorB,
    Topology::kmotor_timer,
    Topology::ktimer_speetec1
#ifdef H11
    ,
    Topology::ktimer_speetec2,
    Topology::kHall_Supply_A,
    Topology::kHall_Supply_B,
    Topology::kSpeetecSupply
#endif
    >;
namespace Types {
// Motor
using MotorTimer = ST_LIB::TimerWrapper<Topology::kmotor_timer>;
using PhaseU =
    decltype(std::declval<MotorTimer>()
                 .template get_dual_pwm<Topology::kMotorPhaseU_P, Topology::kMotorPhaseU_N>());
using PhaseV =
    decltype(std::declval<MotorTimer>()
                 .template get_dual_pwm<Topology::kMotorPhaseV_P, Topology::kMotorPhaseV_N>());
using PhaseW =
    decltype(std::declval<MotorTimer>()
                 .template get_dual_pwm<Topology::kMotorPhaseW_P, Topology::kMotorPhaseW_N>());
using BufferEnable =
    decltype(Devices::
                 DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kBufferEnable>()
    );

using SynchronousMotor = Devices::ThreePhaseMotor<
    BCUBoard,
    BCU::Types::MotorTimer,
    BCU::Types::PhaseU,
    BCU::Types::PhaseV,
    BCU::Types::PhaseW,
    BCU::Types::BufferEnable>;
#ifdef H11
// Current Sensor
using HallSupply_A =
    decltype(Devices::
                 DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kHall_Supply_A>()
    );
using HallSupply_B =
    decltype(Devices::
                 DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kHall_Supply_B>()
    );
#endif
using CurrentSenseA = Sensor<
    BCUBoard,
    float,
    Types::CurrentSense_Data,
#ifdef H11
    Types::HallSupply_A,
#else
    Devices::NoSupply,
#endif
    HardwareConf::FilterSizeCurrent,
    HardwareConf::currentSense_A,
    Topology::kCurrentSenseU_A,
    BCU::Topology::kCurrentSenseV_A,
    Topology::kCurrentSenseW_A>;
using CurrentSenseB = Sensor<
    BCUBoard,
    float,
    Types::CurrentSense_Data,
#ifdef H11
    Types::HallSupply_B,
#else
    Devices::NoSupply,
#endif
    HardwareConf::FilterSizeCurrent,
    HardwareConf::currentSense_B,
    Topology::kCurrentSenseU_B,
    Topology::kCurrentSenseV_B,
    Topology::kCurrentSenseW_B>;

// Voltage Sensors
using VoltageSense = Sensor<
    BCUBoard,
    float,
    Types::VoltageSense_Data,
    Devices::NoSupply,
    HardwareConf::FilterSizeVoltage,
    HardwareConf::voltageSense,
    Topology::kVoltageSensorA,
    Topology::kVoltageSensorB>;

// Temperature Sensor
using TempSense = Sensor<
    BCUBoard,
    float,
    Types::TempSense_Data,
    Devices::NoSupply,
    HardwareConf::FilterSizeTemp,
    HardwareConf::tempSense,
    Topology::kTempSensorA,
    Topology::kTempSensorB>;

// Encoder
#ifdef H11
using SpeetecSupply =
    decltype(Devices::
                 DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kSpeetecSupply>()
    );
using EncoderTimer2 = ST_LIB::TimerWrapper<Topology::ktimer_speetec2>;
using Speetec2 =
    Devices::Speetec<BCUBoard, EncoderTimer2, HardwareConf::samples_speetec, SpeetecSupply>;
#endif
using EncoderTimer1 = ST_LIB::TimerWrapper<Topology::ktimer_speetec1>;

using Speetec1 = Devices::Speetec<
    BCUBoard,
    EncoderTimer1,
    HardwareConf::samples_speetec,
#ifdef H11
    SpeetecSupply
#else
    Devices::NoSupply
#endif
    >;

// Inverter
using InverterA = Devices::
    Inverter_Feedback<BCUBoard, Types::Inverter_Data, Topology::kPPUReady_A, Topology::kPPUFault_A>;
using InverterB = Devices::
    Inverter_Feedback<BCUBoard, Types::Inverter_Data, Topology::kPPUReady_B, Topology::kPPUFault_B>;

// Leds Types:
using LedConnecting =
    decltype(Devices::
                 DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kLED_Connecting>(
                 ));
using LedOperational = decltype(Devices::DigitalOutputWrapper<
                                BCUBoard,
                                Devices::OLogic::N_OPEN,
                                Topology::kLED_Operational>());
using LedFault =
    decltype(Devices::DigitalOutputWrapper<BCUBoard, Devices::OLogic::N_OPEN, Topology::kLED_Fault>(
    ));
using LedSpaceVector = decltype(Devices::DigitalOutputWrapper<
                                BCUBoard,
                                Devices::OLogic::N_OPEN,
                                Topology::kLED_SpaceVector>());
using LedCurrentControl = decltype(Devices::DigitalOutputWrapper<
                                   BCUBoard,
                                   Devices::OLogic::N_OPEN,
                                   Topology::kLED_CurrentControl>());
using LedSpeedControl = decltype(Devices::DigitalOutputWrapper<
                                 BCUBoard,
                                 Devices::OLogic::N_OPEN,
                                 Topology::kLED_SpeedControl>());

struct TelemetryData {
    const Devices::ThreePhaseMotorDefs::Data& motor;
    const VoltageSense_Data& VoltageSense;
    const CurrentSense_Data& currentSenseA;
    const CurrentSense_Data& currentSenseB;
    const Devices::SpeetecDefs::Data& speetec1;
#ifdef H11
    const Devices::SpeetecDefs::Data& speetec2;
#endif
};
using Ethernet = decltype(&BCUBoard::template instance_of<Topology::eth>());
} // namespace Types

} // namespace BCU
