#pragma once
#include "BCU/Topology/Topology.hpp"
#include "BCU/Leds/Leds.hpp"
#include "BCU/Comms/Comms.hpp"
#include "BCU/MotionControl/CurrentController.hpp"
#include "BCU/MotionControl/SpaceVectorModulator.hpp"
#include "BCU/MotionControl/SpeedController.hpp"
namespace BCU {
class Controller {
public:
    struct MotorKey {
        friend class StateMachine;
        friend class Controller;

    private:
        MotorKey() = default;
    };
    static void update();
    static void init();

    inline static void on_space_vector(MotorKey = {}) {
        Types::DutyCycles duties = SpaceVectorModulator::execute();
        motor_.set_duty_cycle(duties.u, duties.v, duties.w);
    }
    inline static void on_current_control(MotorKey = {}) {
        Types::DutyCycles duties = CurrentController::execute(telemetryData());
        motor_.set_duty_cycle(duties.u, duties.v, duties.w);
    }
    inline static void on_speed_control(MotorKey = {}) {
        CurrentController::set_q_ref(SpeedController::execute(telemetryData()));
    }

    inline static Types::SynchronousMotor& motor(MotorKey = {}) { return motor_; }
    // Getters for data
    inline static const Types::TempSense_Data& dataTemp() {
        static const Types::TempSense_Data& ref = tempSense_.subscribe();
        return ref;
    }
    inline static const Types::Inverter_Data& dataInverterA() {
        static const Types::Inverter_Data ref = inverterA_.subscribe();
        return ref;
    }
    inline static const Types::Inverter_Data& dataInverterB() {
        static const Types::Inverter_Data& ref = inverterB_.subscribe();
        return ref;
    }
    inline static const Types::TelemetryData& telemetryData() {
        static const Types::TelemetryData& data{
            .motor = motor_.subscribe(),
            .VoltageSense = voltageSense_.subscribe(),
            .currentSenseA = currentSenseA_.subscribe(),
            .currentSenseB = currentSenseB_.subscribe(),
            .speetec1 = speetec1_.subscribe()
#ifdef H11
                ,
            .speetec2 = speetec2_.subscribe()
#endif
        };
        return data;
    }

private:
    inline static Types::SynchronousMotor motor_{};
    inline static Types::CurrentSenseA currentSenseA_{};
    inline static Types::CurrentSenseB currentSenseB_{};
    inline static Types::VoltageSense voltageSense_{};
    inline static Types::TempSense tempSense_{};
    inline static Types::InverterA inverterA_{};
    inline static Types::InverterB inverterB_{};
    inline static Types::Speetec1 speetec1_{
        HardwareConf::counter_distance_m,
        HardwareConf::sample_time_s
    };
#ifdef H11
    inline static Types::Speetec2 speetec2_{
        HardwareConf::counter_distance_m,
        HardwareConf::sample_time_s
    };
#endif
    // Data
    inline static Types::StateMachineData dataStateMachine{};

    static void check_orders_received();
    static void handle_hardware_configuration_orders();
    static void handle_operational_mode_orders();
    Controller() = delete;

    inline static void update_control_telemetry() {
        voltageSense_.read();
        currentSenseA_.read();
        currentSenseB_.read();
    }
    inline static void update_position_telemetry(){
        speetec1_.read();
        #ifdef H11
        speetec2_.read();
        #endif
    }
    inline static void update_auxiliary_telemetry() {
        inverterA_.read();
        inverterB_.read();
        tempSense_.read();
        dataStateMachine.currentGeneralState =
            StateMachine::General_State_Machine.get_current_state();
        dataStateMachine.currentOperationalState =
            StateMachine::Operational_State_Machine.get_current_state();
    }
};
} // namespace BCU
