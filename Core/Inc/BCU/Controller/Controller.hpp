#include "BCU/Topology/Topology.hpp"
#include "BCU/Leds/Leds.hpp"
#include "BCU/Comms/Comms.hpp"

namespace BCU{
    class Controller{
        using GeneralStates = DataPackets::Bcu_general_state;
        using OperationalStates = DataPackets::Bcu_control_state;
        public:
        Controller(){

        }
        private:
        static bool check_stop_motor(){
            if(OrderPackets::Stop_Control_flag == true /*||Other thing*/){
                return true;
            }
            return false;
        }
        

        inline static Types::SynchronousMotor motor_{};
        inline static Types::CurrentSenseA currentSenseA_{};
        inline static Types::CurrentSenseB currentSenseB_{};
        inline static Types::VoltageSense voltageSense_{};
        inline static Types::TempSense    tempSense_{};
        inline static Types::InverterA    inverterA_{};
        inline static Types::InverterB    inverterB_{};
        inline static Types::EncoderTimer1 encoderTimer1_{};
        inline static Types::EncoderTimer2 encoderTimer2_{};

// ---------- General States ------------------------
        static constexpr auto connecting_state = make_state(
            GeneralStates::Connecting,
            Transition<GeneralStates>{
                GeneralStates::Operational,[](){
                    return Comms::is_connected();
                }
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
        Transition<OperationalStates>{ // Transition to TEST_PWM
            OperationalStates::Test_PWM,
            []() { return OrderPackets::Start_Test_PWM_flag; }
        },
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Space_Vector,
            []() { return OrderPackets::Start_Space_Vector_flag; }
        },
        Transition<OperationalStates>{ // Transition to Start Current Control
            OperationalStates::Current_Control,
            []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{ // Transition to Start Speed Control
            OperationalStates::Speed_Control,
            []() { return OrderPackets::Start_Speed_Control_flag; }
        }
        );
        static constexpr auto nested_TestPWM_state = make_state(
        OperationalStates::Test_PWM,
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Idle,
            []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Space_Vector,
            []() { return OrderPackets::Start_Space_Vector_flag; }
        },
        Transition<OperationalStates>{ // Transition to Start Current Control
            OperationalStates::Current_Control,
            []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{ // Transition to Start Speed Control
            OperationalStates::Speed_Control,
            []() { return OrderPackets::Start_Speed_Control_flag; }
        }
        );
          static constexpr auto nested_SpaceVector_state = make_state(
        OperationalStates::Space_Vector,
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Idle,
            []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{ // Transition to Start Current Control
            OperationalStates::Current_Control,
            []() { return OrderPackets::Start_Current_Control_flag; }
        },
        Transition<OperationalStates>{ // Transition to Start Speed Control
            OperationalStates::Speed_Control,
            []() { return OrderPackets::Start_Speed_Control_flag; }
        }
        );

        static constexpr auto nested_CurrentControl_state = make_state(
        OperationalStates::Current_Control,
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Idle,
            []() { return check_stop_motor(); }
        },
        Transition<OperationalStates>{ // Transition to Start Speed Control
            OperationalStates::Speed_Control,
            []() { return OrderPackets::Start_Speed_Control_flag; }
        }
        );
        static constexpr auto nested_SpeedControl_state = make_state(
        OperationalStates::Speed_Control,
        Transition<OperationalStates>{ // Transition to Space Vector
            OperationalStates::Idle,
            []() { return check_stop_motor(); }
        }
        );
        
        static consteval auto create_operational_machine(){
            
        }
    //Create Operational Machine
    static inline constinit auto Operational_State_Machine =
    []() consteval
    {
        auto sm = make_state_machine(OperationalStates::Idle,
            nested_idle_state,
            nested_TestPWM_state,
            nested_SpaceVector_state,
            nested_CurrentControl_state,
            nested_SpeedControl_state
        );
        //-------- ENTER ACTIONS ----------------//
        //IDLE : enter actions
        sm.add_enter_action([](){
            motor_.stop();
        },nested_idle_state);
        // TESTPWM: enter action
        sm.add_enter_action([](){
            Leds::turn_on<Types::LedSpaceVector>();
            Leds::turn_on<Types::LedSpeedControl>();
            Leds::turn_on<Types::LedCurrentControl>();
            motor_.start(Comms::duty_cycle_u_received,Comms::duty_cycle_v_received,
                Comms::duty_cycle_w_received,Comms::modulation_frequency_received,Comms::dead_time_ns_received);
        },nested_TestPWM_state);
        // Space Vector: enter action
        sm.add_enter_action([](){
            Leds::turn_on<Types::LedSpaceVector>();
        },nested_SpaceVector_state);
        //Current Control : enter action    
        sm.add_enter_action([](){
            Leds::turn_on<Types::LedCurrentControl>();
        },nested_CurrentControl_state);
        // Speed Control: enter action
        sm.add_enter_action([](){
            Leds::turn_on<Types::LedSpeedControl>();
        },nested_SpeedControl_state);


        //------EXIT ACTIONS ---------//

        sm.add_exit_action([](){
            motor_.stop();
            Leds::turn_off<Types::LedSpaceVector>();
            Leds::turn_off<Types::LedSpeedControl>();
            Leds::turn_off<Types::LedCurrentControl>();
        },nested_TestPWM_state);
        // Space Vector: exit action
        sm.add_exit_action([](){
            motor_.stop();
            Leds::turn_off<Types::LedSpaceVector>();
        },nested_SpaceVector_state);
        //Current Control : exit action    
        sm.add_exit_action([](){
            motor_.stop();
            Leds::turn_off<Types::LedCurrentControl>();
        },nested_CurrentControl_state);
        // Speed Control: exit action
        sm.add_exit_action([](){
            motor_.stop();
            Leds::turn_off<Types::LedSpeedControl>();
        },nested_SpeedControl_state);

        //----- Cyclic Actions -------//
        return sm;
        
    }();

    static inline constinit auto PCU_State_Machine = []() consteval
{
    auto nested = StateMachineHelper::add_nesting(operational_state, Operational_State_Machine);
    auto sm = make_state_machine(GeneralStates::Connecting,
        StateMachineHelper::add_nested_machines(nested),
        connecting_state,
        operational_state,
        fault_state
    );
    using namespace std::chrono_literals;
      //-------- ENTER ACTIONS ----------------//
        //Operational: enter action
        sm.add_enter_action([](){
            Leds::turn_on<Types::LedOperational>();
        },operational_state);
        // Fault: enter action
        sm.add_enter_action([](){
            motor_.stop();
            ProtectionManager::propagate_fault();
            Leds::turn_on<Types::LedFault>();
        },fault_state);

        //------EXIT ACTIONS ---------//
        //Connecting: exit action
        sm.add_exit_action([](){
            Leds::turn_off<Types::LedConnecting>();
        },connecting_state);

        // Operational: exit action
        sm.add_exit_action([](){
            motor_.stop();
            Leds::turn_off<Types::LedOperational>();
        },operational_state);
        //Fault : exit action    
        sm.add_exit_action([](){
            Leds::turn_off<Types::LedFault>();
        },fault_state);

        //----- Cyclic Actions -------//
        //Connecting: cyclic actions
          sm.add_cyclic_action([]()
        {
            Leds::toggle<Types::LedConnecting>();
        }, ms(500), connecting_state);
        return sm;
}();

     



// static inline constinit auto PCU_State_Machine = []() consteval
// {
//     auto nested = StateMachineHelper::add_nesting(operational_state, Operational_State_Machine);
//     auto sm = make_state_machine(States_PCU::Connecting,
//         StateMachineHelper::add_nested_machines(nested),
//         connecting_state,
//         operational_state,
//         fault_state

//     );
//     using namespace std::chrono_literals;
//     sm.add_cyclic_action([]()
//     {
//         static bool toggle = true;
//         Actuators::set_led_connecting(toggle);
//         toggle = !toggle;
//     }, ms(500), connecting_state);

//     sm.add_enter_action([]()
//     {
//         Actuators::set_led_connecting(true);
//         Actuators::set_led_operational(false);
//         Actuators::set_led_fault(false);
//     }, operational_state);

//     sm.add_exit_action([]()
//     {
//         stop_motors();
//         Actuators::set_led_connecting(false);
//     }, operational_state);

//     sm.add_enter_action([]()
//     {
//         stop_motors();
//         ProtectionManager::propagate_fault();
//         Actuators::set_led_operational(false);
//         Actuators::set_led_connecting(false);
//         Actuators::set_led_fault(true);
//     }, fault_state);


//     return sm;
// }();




//     }
// );
    };

}