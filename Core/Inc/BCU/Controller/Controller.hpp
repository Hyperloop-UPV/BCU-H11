#pragma once
#include "BCU/Topology/Topology.hpp"
#include "BCU/Leds/Leds.hpp"
#include "BCU/Comms/Comms.hpp"
#include "BCU/MotionControl/CurrentController.hpp"
#include "BCU/MotionControl/SpaceVectorModulator.hpp"
#include "BCU/MotionControl/SpeedController.hpp"
namespace BCU {
class Controller {
    friend class StateMachine;
public:
    static void update();
    static void init();      
private:
    static void check_orders_received();
    Controller() = delete;
    static void update_control_telemetry();
    static void update_auxiliary_telemetry();
    static inline void on_current_control();
    static inline void on_space_vector();
    static inline void on_speed_control();
    static bool check_stop_motor();

    inline static Types::SynchronousMotor motor_{};
    inline static Types::CurrentSenseA currentSenseA_{};
    inline static Types::CurrentSenseB currentSenseB_{};
    inline static Types::VoltageSense voltageSense_{};
    inline static Types::TempSense tempSense_{};
    inline static Types::InverterA inverterA_{};
    inline static Types::InverterB inverterB_{};
    inline static Types::Speetec1 speetec1_{ HardwareConf::counter_distance_m,
            HardwareConf::sample_time_s};
#ifdef H11
    inline static Types::Speetec2 speetec2_{ HardwareConf::counter_distance_m,
            HardwareConf::sample_time_s};
#endif
    // Data
    inline static Types::StateMachineData dataStateMachine{};

    static auto& dataTemp();
    static auto& dataInverterA();
    static auto& dataInverterB();
    static auto& telemetryData();
};
}