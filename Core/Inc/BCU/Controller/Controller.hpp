#pragma once
#include "BCU/Topology/Topology.hpp"
#include "BCU/Leds/Leds.hpp"
#include "BCU/Comms/Comms.hpp"
#include "BCU/MotionControl/CurrentController.hpp"
#include "BCU/MotionControl/SpaceVectorModulator.hpp"
#include "BCU/MotionControl/SpeedController.hpp"
namespace BCU {
class Controller {
    using GeneralStates = DataPackets::Bcu_general_state;
    using OperationalStates = DataPackets::Bcu_operational_state;

public:
    static inline void update() {
        Scheduler::update();
        Comms::update();
        BCU_State_Machine.check_transitions();
        Operational_State_Machine.check_transitions();
        check_orders_received();
    }
    static inline void init() {
        BCUBoard::init();
        motor_ = std::make_unique<Types::SynchronousMotor>();
        currentSenseA_ = std::make_unique<Types::CurrentSenseA>();
        currentSenseB_ = std::make_unique<Types::CurrentSenseB>();
        voltageSense_ = std::make_unique<Types::VoltageSense>();
        tempSense_ = std::make_unique<Types::TempSense>();
        inverterA_ = std::make_unique<Types::InverterA>();
        inverterB_ = std::make_unique<Types::InverterB>();
        speetec1_ = std::make_unique<Types::Speetec1>(
            HardwareConf::counter_distance_m,
            HardwareConf::sample_time_s
        );
#ifdef H11
        speetec2_ = std::make_unique<Types::Speetec2>(
            HardwareConf::counter_distance_m,
            HardwareConf::sample_time_s
        );
#endif
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
        voltageSense_->turn_on();
        currentSenseA_->turn_on();
        currentSenseB_->turn_on();
        speetec1_->turn_on();
#ifdef H11
        speetec2_->turn_on();
#endif
        BCU_State_Machine.start();
        Operational_State_Machine.start();
        // Create cyclic actions
        Scheduler::register_task(
            ControlConf::TelemetryDataAuxiliarPeriod,
            &update_auxiliary_telemetry
        );
        Scheduler::register_task(
            ControlConf::TelemetryDataControlPeriod,
            &update_control_telemetry
        );
    }

private:
    static inline void check_orders_received() {

        if (OrderPackets::Configure_Commutation_Parameters_flag) {
            OrderPackets::Configure_Commutation_Parameters_flag = false;
            motor_->set_frequency(Comms::commutation_frequency_received);
            motor_->set_dead_time(Comms::dead_time_ns_received);
        }
        if (OrderPackets::Change_angle_offset_flag) {
            OrderPackets::Change_angle_offset_flag = false;
            CurrentController::set_angle_offset(Comms::angle_offset_received);
        }
        OperationalStates state = Operational_State_Machine.get_current_state();
        // This will only be executed inside test PWM
        if (OrderPackets::Start_Test_PWM_flag) {
            OrderPackets::Start_Test_PWM_flag = false;
            if (state == OperationalStates::Test_PWM) {
                motor_->setup_signals(
                    Comms::duty_cycle_u_received,
                    Comms::duty_cycle_v_received,
                    Comms::duty_cycle_w_received,
                    Comms::commutation_frequency_received
                );
            }
        } else if (OrderPackets::Start_Space_Vector_flag) {
            OrderPackets::Start_Space_Vector_flag = false;
            if (state == OperationalStates::Space_Vector) {
                motor_->set_frequency(Comms::commutation_frequency_received);
                SpaceVectorModulator::set_modulation_freq(Comms::modulation_frequency_received);
                SpaceVectorModulator::set_modulation_index(
                    Comms::voltage_reference_received,
                    Comms::voltage_max_received
                );
            }
        } else if (OrderPackets::Start_Current_Control_flag && state == OperationalStates::Current_Control) {
            OrderPackets::Start_Current_Control_flag = false;
            if (state == OperationalStates::Current_Control) {
                motor_->set_frequency(Comms::commutation_frequency_received);
                CurrentController::set_d_ref(Comms::d_current_reference_received);
                CurrentController::set_q_ref(Comms::q_current_reference_received);
                CurrentController::reset();
            }
        } else if (OrderPackets::Start_Speed_Control_flag) {
            OrderPackets::Start_Speed_Control_flag = false;
            if (state == OperationalStates::Speed_Control) {
                SpeedController::set_speed_m_s(Comms::target_linear_speed_received);
            }
        }
    }
    Controller() = delete;
    static inline void update_control_telemetry() {
        voltageSense_->read();
        currentSenseA_->read();
        currentSenseB_->read();
        speetec1_->read();
#ifdef H11
        speetec2_->read();
#endif
        // read encoder
    }
    static inline void update_auxiliary_telemetry() {
        inverterA_->read();
        inverterB_->read();
        tempSense_->read();
        dataStateMachine.currentGeneralState = BCU_State_Machine.get_current_state();
        dataStateMachine.currentOperationalState = Operational_State_Machine.get_current_state();
    }
    static inline void on_current_control() {
        update_control_telemetry();
        dataStateMachine.currentOperationalState = Operational_State_Machine.get_current_state();
        if (dataStateMachine.currentOperationalState == OperationalStates::Current_Control ||
            dataStateMachine.currentOperationalState == OperationalStates::Speed_Control) {
            Types::DutyCycles duties = CurrentController::execute(telemetryData());
            motor_->set_duty_cycle(duties.u, duties.v, duties.w);
        }
    }
    static inline void on_space_vector() {
        if (Operational_State_Machine.get_current_state() == OperationalStates::Space_Vector) {
            Types::DutyCycles duties = SpaceVectorModulator::execute();
            motor_->set_duty_cycle(duties.u, duties.v, duties.w);
        }
    }
    static inline void on_speed_control() {
        if (Operational_State_Machine.get_current_state() == OperationalStates::Speed_Control) {
            CurrentController::set_q_ref(SpeedController::execute(telemetryData()));
        }
    }
    static bool check_stop_motor() {
        if (OrderPackets::Stop_Control_flag == true /*||Other thing*/) {
            return true;
        }
        return false;
    }

    inline static std::unique_ptr<Types::SynchronousMotor> motor_;
    inline static std::unique_ptr<Types::CurrentSenseA> currentSenseA_;
    inline static std::unique_ptr<Types::CurrentSenseB> currentSenseB_;
    inline static std::unique_ptr<Types::VoltageSense> voltageSense_;
    inline static std::unique_ptr<Types::TempSense> tempSense_;
    inline static std::unique_ptr<Types::InverterA> inverterA_;
    inline static std::unique_ptr<Types::InverterB> inverterB_;
    inline static std::unique_ptr<Types::Speetec1> speetec1_;
#ifdef H11
    inline static std::unique_ptr<Types::Speetec2> speetec2_;
#endif
    // Data
    inline static Types::StateMachineData dataStateMachine{};

    inline static const Types::TempSense_Data& dataTemp() {
        static const Types::TempSense_Data& ref = tempSense_->subscribe();
        return ref;
    }
    inline static auto& dataInverterA() {
        static auto& ref = inverterA_->subscribe();
        return ref;
    }
    inline static auto& dataInverterB() {
        static auto& ref = inverterB_->subscribe();
        return ref;
    }
    static Types::TelemetryData& telemetryData() {
        static Types::TelemetryData data{
            .motor = motor_->subscribe(),
            .VoltageSense = voltageSense_->subscribe(),
            .currentSenseA = currentSenseA_->subscribe(),
            .currentSenseB = currentSenseB_->subscribe(),
            .speetec1 = speetec1_->subscribe()
#ifdef H11
                ,
            .speetec2 = speetec2_->subscribe()
#endif
        };
        return data;
    }
    // ---------- General States ------------------------
    static constexpr auto connecting_state = make_state(
        GeneralStates::Connecting,
        Transition<GeneralStates>{
            GeneralStates::Operational,
            []() { return Comms::is_connected(); }
        }
    );

    static constexpr auto operational_state = make_state(
        GeneralStates::Operational,
        Transition<GeneralStates>{
            GeneralStates::Fault,
            []() {
                return !Comms::is_connected(); // || other things
            }
        }
    );
    static constexpr auto fault_state = make_state(GeneralStates::Fault);

    // ----   Control States (Inside Operational State Machine) ------//
    static constexpr auto nested_idle_state = make_state(
        OperationalStates::Idle,
        Transition<OperationalStates>{// Transition to TEST_PWM
                                      OperationalStates::Test_PWM,
                                      []() { return OrderPackets::Start_Test_PWM_flag; }
        },
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Space_Vector,
                                      []() { return OrderPackets::Start_Space_Vector_flag; }
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      []() { return OrderPackets::Start_Speed_Control_flag; }
        }
    );
    static constexpr auto nested_TestPWM_state = make_state(
        OperationalStates::Test_PWM,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Space_Vector,
                                      []() { return OrderPackets::Start_Space_Vector_flag; }
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      []() { return OrderPackets::Start_Speed_Control_flag; }
        }
    );
    static constexpr auto nested_SpaceVector_state = make_state(
        OperationalStates::Space_Vector,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      []() { return OrderPackets::Start_Speed_Control_flag; }
        }
    );

    static constexpr auto nested_CurrentControl_state = make_state(
        OperationalStates::Current_Control,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      []() { return OrderPackets::Start_Speed_Control_flag; }
        }
    );
    static constexpr auto nested_SpeedControl_state = make_state(
        OperationalStates::Speed_Control,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      []() { return check_stop_motor(); }
        }
    );
    //-----------Operational States-----------------//

    //---- Enter Actions  ---------//
    static void enter_idle_state() {
        OrderPackets::Stop_Control_flag = false;
        motor_->stop();
    }
    static void enter_TestPWM_state() {
        OrderPackets::Start_Test_PWM_flag = false;
        Leds::turn_on<Types::LedSpaceVector>();
        Leds::turn_on<Types::LedSpeedControl>();
        Leds::turn_on<Types::LedCurrentControl>();
        motor_->start(
            Comms::duty_cycle_u_received,
            Comms::duty_cycle_v_received,
            Comms::duty_cycle_w_received,
            Comms::commutation_frequency_received
        );
    }
    static void enter_SpaceVector_state() {
        OrderPackets::Start_Current_Control_flag = false;
        motor_->set_frequency(Comms::commutation_frequency_received);
        motor_->engage();
        SpaceVectorModulator::set_modulation_freq(Comms::modulation_frequency_received);
        SpaceVectorModulator::set_modulation_index(
            Comms::voltage_reference_received,
            Comms::voltage_max_received
        );
        Leds::turn_on<Types::LedSpaceVector>();
    }
    static void enter_CurrentControl_state() {
        OrderPackets::Start_Current_Control_flag = false;
        motor_->set_frequency(Comms::commutation_frequency_received);
        motor_->engage();
        CurrentController::set_d_ref(Comms::d_current_reference_received);
        CurrentController::set_q_ref(Comms::q_current_reference_received);
        CurrentController::reset();
        Leds::turn_on<Types::LedCurrentControl>();
    }
    static void enter_SpeedControl_state() {
        OrderPackets::Start_Speed_Control_flag = false;
        SpeedController::set_speed_m_s(Comms::target_linear_speed_received);
        motor_->engage();
        Leds::turn_on<Types::LedSpeedControl>();
    }
    //--
    //----Exit Actions ------------------------//
    static void exit_TestPWM_state() {
        motor_->stop();
        Leds::turn_off<Types::LedSpaceVector>();
        Leds::turn_off<Types::LedSpeedControl>();
        Leds::turn_off<Types::LedCurrentControl>();
    }
    static void exit_SpaceVector_state() {
        motor_->stop();
        Leds::turn_off<Types::LedSpaceVector>();
    }
    static void exit_CurrentControl_state() {
        motor_->stop();
        Leds::turn_off<Types::LedCurrentControl>();
    }
    static void exit_SpeedControl_state() {
        motor_->stop();
        Leds::turn_off<Types::LedSpeedControl>();
    }

    // ------------- General State Machine ---------------------//
    //------------Enter Actions --------------------------//
    static void enter_Operational_state() { Leds::turn_on<Types::LedOperational>(); }
    static void enter_Fault_state() {
        motor_->stop();
        ProtectionManager::propagate_fault();
        Leds::turn_on<Types::LedFault>();
    }
    //----------------- Exit Actions ----------------------//
    static void exit_Connecting_state() { Leds::turn_off<Types::LedConnecting>(); }
    static void exit_Operational_state() {
        motor_->stop();
        Leds::turn_off<Types::LedOperational>();
    }
    static void exit_Fault_state() { Leds::turn_off<Types::LedFault>(); }

    static void Connecting_cyclic_action() { Leds::toggle<Types::LedConnecting>(); }

    // Create Operational Machine
    static inline constinit auto Operational_State_Machine = []() consteval {
        auto sm = make_state_machine(
            OperationalStates::Idle,
            nested_idle_state,
            nested_TestPWM_state,
            nested_SpaceVector_state,
            nested_CurrentControl_state,
            nested_SpeedControl_state
        );
        //-------- ENTER ACTIONS ----------------//
        // IDLE : enter actions
        sm.add_enter_action(&enter_idle_state, nested_idle_state);
        // TESTPWM: enter action
        sm.add_enter_action(&enter_TestPWM_state, nested_TestPWM_state);
        // Space Vector: enter action
        sm.add_enter_action(&enter_SpaceVector_state, nested_SpaceVector_state);
        // Current Control : enter action
        sm.add_enter_action(&enter_CurrentControl_state, nested_CurrentControl_state);
        // Speed Control: enter action
        sm.add_enter_action(&enter_SpeedControl_state, nested_SpeedControl_state);

        //------EXIT ACTIONS ---------//

        sm.add_exit_action(&exit_TestPWM_state, nested_TestPWM_state);
        // Space Vector: exit action
        sm.add_exit_action(&exit_SpaceVector_state, nested_SpaceVector_state);
        // Current Control : exit action
        sm.add_exit_action(&exit_CurrentControl_state, nested_CurrentControl_state);
        // Speed Control: exit action
        sm.add_exit_action(&exit_SpeedControl_state, nested_SpeedControl_state);

        //-----Cyclic Actions -------//
        sm.add_cyclic_action(
            &on_space_vector,
            ControlConf::SpaceVectorPeriodTime,
            nested_SpaceVector_state
        );
        sm.add_cyclic_action(
            &on_current_control,
            ControlConf::CurrentControlPeriodTime,
            nested_CurrentControl_state
        );
        sm.add_cyclic_action(
            &on_speed_control,
            ControlConf::SpeedControlPeriodTime,
            nested_SpeedControl_state
        );
        return sm;
    }();

    static inline constinit auto BCU_State_Machine = []() consteval {
        auto nested = StateMachineHelper::add_nesting(operational_state, Operational_State_Machine);
        auto sm = make_state_machine(
            GeneralStates::Connecting,
            StateMachineHelper::add_nested_machines(nested),
            connecting_state,
            operational_state,
            fault_state
        );

        //-------- ENTER ACTIONS ----------------//
        // Operational: enter action
        sm.add_enter_action(&enter_Operational_state, operational_state);
        // Fault: enter action
        sm.add_enter_action(&enter_Fault_state, fault_state);

        //------EXIT ACTIONS ---------//
        // Connecting: exit action
        sm.add_exit_action(&exit_Connecting_state, connecting_state);

        // Operational: exit action
        sm.add_exit_action(&exit_Operational_state, operational_state);
        // Fault : exit action
        sm.add_exit_action(&exit_Fault_state, fault_state);

        //----- Cyclic Actions -------//
        // Connecting: cyclic actions
        sm.add_cyclic_action(
            &Connecting_cyclic_action,
            ControlConf::Connecting_Cyclic_action,
            connecting_state
        );
        return sm;
    }();
};

} // namespace BCU
