#include "BCU/StateMachine/StateMachine.hpp"
#include "BCU/Controller/Controller.hpp"

namespace BCU {
    void StateMachine::enter_idle_state() {
        OrderPackets::Stop_Control_flag = false;
        Controller::motor_.stop();
    }
     void StateMachine::enter_TestPWM_state() {
        OrderPackets::Start_Test_PWM_flag = false;
        Leds::turn_on<Types::LedSpaceVector>();
        Leds::turn_on<Types::LedSpeedControl>();
        Leds::turn_on<Types::LedCurrentControl>();
        Controller::motor_.start(
            Comms::duty_cycle_u_received,
            Comms::duty_cycle_v_received,
            Comms::duty_cycle_w_received,
            Comms::commutation_frequency_received
        );
    }
     void StateMachine::enter_SpaceVector_state() {
        OrderPackets::Start_Current_Control_flag = false;
        Controller::motor_.set_frequency(Comms::commutation_frequency_received);
        Controller::motor_.engage();
        SpaceVectorModulator::set_modulation_freq(Comms::modulation_frequency_received);
        SpaceVectorModulator::set_modulation_index(
            Comms::voltage_reference_received,
            Comms::voltage_max_received
        );
        Leds::turn_on<Types::LedSpaceVector>();
    }
     void StateMachine::enter_CurrentControl_state() {
        OrderPackets::Start_Current_Control_flag = false;
        Controller::motor_.set_frequency(Comms::commutation_frequency_received);
        Controller::motor_.engage();
        CurrentController::set_d_ref(Comms::d_current_reference_received);
        CurrentController::set_q_ref(Comms::q_current_reference_received);
        CurrentController::reset();
        Leds::turn_on<Types::LedCurrentControl>();
    }
     void StateMachine::enter_SpeedControl_state() {
        OrderPackets::Start_Speed_Control_flag = false;
        SpeedController::set_speed_m_s(Comms::target_linear_speed_received);
        Controller::motor_.engage();
        Leds::turn_on<Types::LedSpeedControl>();
    }
    //--
    //----Exit Actions ------------------------//
     void StateMachine::exit_TestPWM_state() {
        Controller::motor_.stop();
        Leds::turn_off<Types::LedSpaceVector>();
        Leds::turn_off<Types::LedSpeedControl>();
        Leds::turn_off<Types::LedCurrentControl>();
    }
     void StateMachine::exit_SpaceVector_state() {
        Controller::motor_.stop();
        Leds::turn_off<Types::LedSpaceVector>();
    }
     void StateMachine::exit_CurrentControl_state() {
        Controller::motor_.stop();
        Leds::turn_off<Types::LedCurrentControl>();
    }
     void StateMachine::exit_SpeedControl_state() {
        Controller::motor_.stop();
        Leds::turn_off<Types::LedSpeedControl>();
    }

    // ------------- General State Machine ---------------------//


    //------------Enter Actions --------------------------//
     void StateMachine::enter_Operational_state() { Leds::turn_on<Types::LedOperational>(); }
     void StateMachine::enter_Fault_state() {
        Controller::motor_.stop();
        Leds::turn_on<Types::LedFault>();
    }
    //----------------- Exit Actions ----------------------//
     void StateMachine::exit_Connecting_state() { Leds::turn_off<Types::LedConnecting>(); }
     void StateMachine::exit_Operational_state() {
        Controller::motor_.stop();
        Leds::turn_off<Types::LedOperational>();
    }
     void StateMachine::Connecting_cyclic_action() { Leds::toggle<Types::LedConnecting>(); }


     void StateMachine::on_space_vector(){
        Controller
     }
}