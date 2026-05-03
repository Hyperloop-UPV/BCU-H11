#include "BCU/Controller/Controller.hpp"
namespace BCU {
void Controller::update() {
    Scheduler::update();
    Comms::update();
    StateMachine::General_State_Machine.check_transitions();
    StateMachine::Operational_State_Machine.check_transitions();
    check_orders_received();
}
void Controller::init() {
    BCUBoard::init();
    // Link Communications
    Comms::init<
        Devices::ThreePhaseMotorDefs::Data,
        Types::CurrentSense_Data,
        Types::VoltageSense_Data,
        Types::TempSense_Data,
        Devices::SpeetecDefs::Data>(
        telemetryData().motor,
        telemetryData().currentSenseA,
        telemetryData().currentSenseB,
        telemetryData().VoltageSense,
        dataTemp(),
        telemetryData().speetec1,
#ifdef H11
        telemetryData().speetec2,
#else
        telemetryData().speetec1,
#endif
        dataStateMachine
    );

    // turn on sensors
    voltageSense_.turn_on();
    currentSenseA_.turn_on();
    currentSenseB_.turn_on();
    speetec1_.turn_on();
#ifdef H11
    speetec2_.turn_on();
#endif
    StateMachine::General_State_Machine.start();
    StateMachine::Operational_State_Machine.start();
    // Create cyclic actions
    Scheduler::register_task(ControlConf::TelemetryDataAuxiliarPeriod, &update_auxiliary_telemetry);
    Scheduler::register_task(ControlConf::TelemetryDataControlPeriod, &update_control_telemetry);
}
void Controller::check_orders_received() {
    handle_hardware_configuration_orders();
    handle_operational_mode_orders();
}

void Controller::handle_hardware_configuration_orders() {
    if (OrderPackets::Configure_Commutation_Parameters_flag) {
        OrderPackets::Configure_Commutation_Parameters_flag = false;
        motor_.set_frequency(Comms::commutation_frequency_received);
        motor_.set_dead_time(Comms::dead_time_ns_received);
    }

    if (OrderPackets::Change_angle_offset_flag) {
        OrderPackets::Change_angle_offset_flag = false;
        CurrentController::set_angle_offset(Comms::angle_offset_received);
    }
}

void Controller::handle_operational_mode_orders() {
    auto state = StateMachine::Operational_State_Machine.get_current_state();

    if (OrderPackets::Start_Test_PWM_flag && state == OperationalStates::Test_PWM) {
        OrderPackets::Start_Test_PWM_flag = false;
        motor_.setup_signals(
            Comms::duty_cycle_u_received,
            Comms::duty_cycle_v_received,
            Comms::duty_cycle_w_received,
            Comms::commutation_frequency_received
        );
    }
    if (OrderPackets::Start_Space_Vector_flag && state == OperationalStates::Space_Vector) {
        OrderPackets::Start_Space_Vector_flag = false;
        motor_.set_frequency(Comms::commutation_frequency_received);
        SpaceVectorModulator::set_modulation_freq(Comms::modulation_frequency_received);
        SpaceVectorModulator::set_modulation_index(
            Comms::voltage_reference_received,
            Comms::voltage_max_received
        );
    }
    if (OrderPackets::Start_Current_Control_flag && state == OperationalStates::Current_Control) {
        OrderPackets::Start_Current_Control_flag = false;
        motor_.set_frequency(Comms::commutation_frequency_received);
        CurrentController::set_d_ref(Comms::d_current_reference_received);
        CurrentController::set_q_ref(Comms::q_current_reference_received);
        CurrentController::reset();
    }
    if (OrderPackets::Start_Speed_Control_flag && state == OperationalStates::Speed_Control) {
        OrderPackets::Start_Speed_Control_flag = false;
        SpeedController::set_speed_m_s(Comms::target_linear_speed_received);
    }
}
} // namespace BCU
