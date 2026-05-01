#pragma once
#include "BCU/Data/Data.hpp"

namespace BCU{
    using GeneralStates = DataPackets::Bcu_general_state;
    using OperationalStates = DataPackets::Bcu_operational_state;

    class StateMachine {
    private:
    
    //transitions
    static bool trans_to_operational();
    static bool trans_to_fault();
    static bool trans_to_idle();
    static bool trans_to_testPWM();
    static bool trans_to_spaceVector();
    static bool trans_to_currentControl();
    static bool trans_to_speedControl();

    //enter actions
    static void enter_Connecting_state();
    static void enter_Operational_state();
    
    static void enter_idle_state();
    static void enter_TestPWM_state();
    static void enter_SpaceVector_state();
    static void enter_CurrentControl_state();
    static void enter_SpeedControl_state();
    //exit actions
    static void exit_Connecting_state();
    static void exit_Operational_state();
    
    static void exit_idle_state();
    static void exit_TestPWM_state();
    static void exit_SpaceVector_state();
    static void exit_CurrentControl_state();
    static void exit_SpeedControl_state();
    //cyclic actions
    static void Connecting_cyclic_action();
    static void on_space_vector();
    static void on_current_control();
    static void on_speed_control();

     static constexpr auto operational_state = make_state(
        GeneralStates::Operational,
        Transition<GeneralStates>{
            GeneralStates::Fault,
            &trans_to_fault
        }
    );
    static constexpr auto fault_state = make_state(GeneralStates::Fault);

    // ----   Control States (Inside Operational State Machine) ------//
    static constexpr auto nested_idle_state = make_state(
        OperationalStates::Idle,
        Transition<OperationalStates>{// Transition to TEST_PWM
                                      OperationalStates::Test_PWM,
                                      &trans_to_testPWM 
        },
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Space_Vector,
                                      &trans_to_spaceVector 
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      &trans_to_currentControl 
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      &trans_to_speedControl 
        }
    );
    static constexpr auto nested_TestPWM_state = make_state(
        OperationalStates::Test_PWM,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      &trans_to_idle 
        },
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Space_Vector,
                                      &trans_to_spaceVector 
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      &trans_to_currentControl 
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      &trans_to_speedControl 
        }
    );
    static constexpr auto nested_SpaceVector_state = make_state(
        OperationalStates::Space_Vector,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      &trans_to_idle 
        },
        Transition<OperationalStates>{// Transition to Start Current Control
                                      OperationalStates::Current_Control,
                                      &trans_to_currentControl 
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      &trans_to_speedControl 
        }
    );

    static constexpr auto nested_CurrentControl_state = make_state(
        OperationalStates::Current_Control,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      &trans_to_idle 
        },
        Transition<OperationalStates>{// Transition to Start Speed Control
                                      OperationalStates::Speed_Control,
                                      &trans_to_speedControl 
        }
    );
    static constexpr auto nested_SpeedControl_state = make_state(
        OperationalStates::Speed_Control,
        Transition<OperationalStates>{// Transition to Space Vector
                                      OperationalStates::Idle,
                                      &trans_to_idle 
        }
    );

    public:
    //added enter fault action public so can be called from the fault policy
   static void enter_Fault_state();
    static constexpr auto connecting_state = make_state(
        GeneralStates::Connecting,
        Transition<GeneralStates>{
            GeneralStates::Operational,
            &trans_to_operational }
    );

   
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
        //--Space Vector: cyclic action
        sm.add_cyclic_action(
            &on_space_vector,
            ControlConf::SpaceVectorPeriodTime,
            nested_SpaceVector_state
        );
        // Current Control: cyclic action
        sm.add_cyclic_action(
            &on_current_control,
            ControlConf::CurrentControlPeriodTime,
            nested_CurrentControl_state
        );
        // Speed Control: cyclic action
        sm.add_cyclic_action(
            &on_current_control,
            ControlConf::CurrentControlPeriodTime,
            nested_SpeedControl_state
        );
        sm.add_cyclic_action(
            &on_speed_control,
            ControlConf::SpeedControlPeriodTime,
            nested_SpeedControl_state
        );
        return sm;
    }();

    static inline constinit auto General_State_Machine = []() consteval {
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
}
