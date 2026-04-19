#include "BCU/Topology/Topology.hpp"
#include "BCU/Leds/Leds.hpp"
#include "BCU/Comms/Comms.hpp"

namespace BCU {
class Controller {
    using GeneralStates = DataPackets::Bcu_general_state;
    using OperationalStates = DataPackets::Bcu_control_state;

public:
    Controller() {
        // turn on sensors
        voltageSense_.turn_on();
        currentSenseA_.turn_on();
        currentSenseB_.turn_on();
    }

private:
    static inline void update_vital_telemetry() {
        voltageSense_.read();
        currentSenseA_.read();
        currentSenseB_.read();
        // read encoder
    }
    static inline void update_other_telemetry() {
        inverterA_.read();
        inverterB_.read();
        tempSense_.read();
    }
    static bool check_stop_motor() {
        if (OrderPackets::Stop_Control_flag == true /*||Other thing*/) {
            return true;
        }
        return false;
    }

    inline static Types::SynchronousMotor motor_{};
    inline static Types::CurrentSenseA currentSenseA_{};
    inline static Types::CurrentSenseB currentSenseB_{};
    inline static Types::VoltageSense voltageSense_{};
    inline static Types::TempSense tempSense_{};
    inline static Types::InverterA inverterA_{};
    inline static Types::InverterB inverterB_{};
    inline static Types::EncoderTimer1 encoderTimer1_{};
    inline static Types::EncoderTimer2 encoderTimer2_{};

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
    static constexpr void enter_idle_state() { motor_.stop(); }
    static constexpr void enter_TestPWM_state() {
        Leds::turn_on<Types::LedSpaceVector>();
        Leds::turn_on<Types::LedSpeedControl>();
        Leds::turn_on<Types::LedCurrentControl>();
        motor_.start(
            Comms::duty_cycle_u_received,
            Comms::duty_cycle_v_received,
            Comms::duty_cycle_w_received,
            Comms::modulation_frequency_received,
            Comms::dead_time_ns_received
        );
    }
    static constexpr void enter_SpaceVector_state() { Leds::turn_on<Types::LedSpaceVector>(); }
    static constexpr void enter_CurrentControl_state() {
        Leds::turn_on<Types::LedCurrentControl>();
    }
    static constexpr void enter_SpeedControl_state() { Leds::turn_on<Types::LedSpeedControl>(); }
    //------Exit Actions ------------------------//
    static constexpr void exit_TestPWM_state() {
        motor_.stop();
        Leds::turn_off<Types::LedSpaceVector>();
        Leds::turn_off<Types::LedSpeedControl>();
        Leds::turn_off<Types::LedCurrentControl>();
    }
    static constexpr void exit_SpaceVector_state() {
        motor_.stop();
        Leds::turn_off<Types::LedSpaceVector>();
    }
    static constexpr void exit_CurrentControl_state() {
        motor_.stop();
        Leds::turn_off<Types::LedCurrentControl>();
    }
    static constexpr void exit_SpeedControl_state() {
        motor_.stop();
        Leds::turn_off<Types::LedSpeedControl>();
    }

    // ------------- General State Machine ---------------------//
    //------------Enter Actions --------------------------//
    static constexpr void enter_Operational_state() { Leds::turn_on<Types::LedOperational>(); }
    static constexpr void enter_Fault_state() {
        motor_.stop();
        ProtectionManager::propagate_fault();
        Leds::turn_on<Types::LedFault>();
    }
    //----------------- Exit Actions ----------------------//
    static constexpr void exit_Connecting_state() { Leds::turn_off<Types::LedConnecting>(); }
    static constexpr void exit_Operational_state() {
        motor_.stop();
        Leds::turn_off<Types::LedOperational>();
    }
    static constexpr void exit_Fault_state() { Leds::turn_off<Types::LedFault>(); }

    static constexpr void Connecting_cyclic_action() { Leds::toggle<Types::LedConnecting>(); }

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
        sm.add_enter_action([]() { enter_idle_state(); }, nested_idle_state);
        // TESTPWM: enter action
        sm.add_enter_action([]() { enter_TestPWM_state(); }, nested_TestPWM_state);
        // Space Vector: enter action
        sm.add_enter_action([]() { enter_SpaceVector_state(); }, nested_SpaceVector_state);
        // Current Control : enter action
        sm.add_enter_action([]() { enter_CurrentControl_state(); }, nested_CurrentControl_state);
        // Speed Control: enter action
        sm.add_enter_action([]() { enter_SpeedControl_state(); }, nested_SpeedControl_state);

        //------EXIT ACTIONS ---------//

        sm.add_exit_action([]() { exit_TestPWM_state(); }, nested_TestPWM_state);
        // Space Vector: exit action
        sm.add_exit_action([]() { exit_SpaceVector_state(); }, nested_SpaceVector_state);
        // Current Control : exit action
        sm.add_exit_action([]() { exit_CurrentControl_state(); }, nested_CurrentControl_state);
        // Speed Control: exit action
        sm.add_exit_action([]() { exit_SpeedControl_state(); }, nested_SpeedControl_state);
        return sm;
    }();

    static inline constinit auto PCU_State_Machine = []() consteval {
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
        sm.add_enter_action([]() { enter_Operational_state(); }, operational_state);
        // Fault: enter action
        sm.add_enter_action([]() { enter_Fault_state(); }, fault_state);

        //------EXIT ACTIONS ---------//
        // Connecting: exit action
        sm.add_exit_action([]() { exit_Connecting_state(); }, connecting_state);

        // Operational: exit action
        sm.add_exit_action([]() { exit_Operational_state(); }, operational_state);
        // Fault : exit action
        sm.add_exit_action([]() { exit_Fault_state(); }, fault_state);

        //----- Cyclic Actions -------//
        // Connecting: cyclic actions
        sm.add_cyclic_action(
            []() { Connecting_cyclic_action(); },
            Configuration::Connecting_Cyclic_action,
            connecting_state
        );
        return sm;
    }();
};

} // namespace BCU
