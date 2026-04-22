#pragma once
#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"
#include "BCU/Data/Data.hpp"
#include "BCU/MotionControl/SpaceVectorModulator.hpp"
#include "BCU/MotionControl/CurrentController.hpp"
#include "BCU/MotionControl/SpeedController.hpp"

namespace BCU {
class Comms {
public:
    template <
        typename DataMotor,
        typename DataCurrentSense,
        typename DataVoltageSense,
        typename DataTempSense,
        typename DataSpeetec>
    static void init(
        const DataMotor& dataMotor,
        const DataCurrentSense& dataCurrentSenseA,
        const DataCurrentSense& dataCurrentSenseB,
        const DataVoltageSense& dataVoltage,
        const DataTempSense& dataTemp,
        const DataSpeetec& dataSpeetec1,
        const DataSpeetec& dataSpeetec2,
        Types::StateMachineData& dataStateMachine
    ) {
        // The Datapackets need no_const variables, is safe because the code does not make any
        // modification in the variables.
        auto& motor = const_cast<DataMotor&>(dataMotor);
        [[maybe_unused]] auto& CurrentSenseA = const_cast<DataCurrentSense&>(dataCurrentSenseA);
        [[maybe_unused]] auto& CurrentSenseB = const_cast<DataCurrentSense&>(dataCurrentSenseB);
        [[maybe_unused]] auto& Voltage = const_cast<DataVoltageSense&>(dataVoltage);
        [[maybe_unused]] auto& Temp = const_cast<DataTempSense&>(dataTemp);
        [[maybe_unused]] auto& Speetec1 = const_cast<DataSpeetec&>(dataSpeetec1);
        [[maybe_unused]] auto& Speetec2 = const_cast<DataSpeetec&>(dataSpeetec2);
        auto current_control = CurrentController::subscribe();
        auto speed_control = SpeedController::subscribe();
        auto space_vector = SpaceVectorModulator::subscribe();
        DataPackets::Motor_Info_init(
            motor.duty_cycle_u,
            motor.duty_cycle_v,
            motor.duty_cycle_w,
            motor.frequency,
            motor.dead_time_ns,
            reinterpret_cast<DataPackets::Motor_State&>(motor.raw_state)
        );
        DataPackets::State_init(
            dataStateMachine.currentGeneralState,
            dataStateMachine.currentOperationalState
        );
        DataPackets::Space_Vector_init(
            space_vector.modulation_frequency,
            space_vector.modulation_index
        );
        DataPackets::DC_Link_init(Voltage.A, Voltage.B);
        DataPackets::Motor_Currents_init(
            CurrentSenseA.U,
            CurrentSenseA.V,
            CurrentSenseA.W,
            CurrentSenseB.U,
            CurrentSenseB.V,
            CurrentSenseB.W
        );
        DataPackets::SPEETEC_1_init(
            Speetec1.position,
            reinterpret_cast<DataPackets::direction_speetec&>(Speetec1.raw_direction),
            Speetec1.speed,
            Speetec1.acceleration
        );
        DataPackets::SPEETEC_2_init(
            Speetec2.position,
            reinterpret_cast<DataPackets::direction_speetec&>(Speetec2.raw_direction),
            Speetec2.speed,
            Speetec2.acceleration
        );
        DataPackets::Current_Control_init(
            current_control.reference.d,
            current_control.measured.d,
            current_control.error.d,
            current_control.reference.q,
            current_control.measured.q,
            current_control.error.q,
            current_control.electrical_angle,
            current_control.output.d,
            current_control.output.q,
            current_control.angle_offset,
            current_control.target.u,
            current_control.target.v,
            current_control.target.w
        );
        DataPackets::Speed_Control_init(
            speed_control.target_speed,
            speed_control.output_iq_ref,
            speed_control.error_speed
        );

        OrderPackets::Stop_Control_init();
        OrderPackets::Start_Test_PWM_init(
            commutation_frequency_received,
            duty_cycle_u_received,
            duty_cycle_v_received,
            duty_cycle_w_received
        );
        OrderPackets::Configure_Commutation_Parameters_init(
            commutation_frequency_received,
            dead_time_ns_received
        );
        OrderPackets::Start_Space_Vector_init(
            commutation_frequency_received,
            modulation_frequency_received,
            voltage_reference_received,
            voltage_max_received
        );
        OrderPackets::Start_Current_Control_init(
            commutation_frequency_received,
            d_current_reference_received,
            q_current_reference_received
        );
        OrderPackets::Start_Speed_Control_init(target_linear_speed_received);
        OrderPackets::Change_angle_offset_init(angle_offset_received);
        DataPackets::start();
        OrderPackets::start();
    }
    inline static void clear_flags() {
        OrderPackets::Stop_Control_flag = false;
        OrderPackets::Start_Test_PWM_flag = false;
        OrderPackets::Configure_Commutation_Parameters_flag = false;
        OrderPackets::Start_Space_Vector_flag = false;
        OrderPackets::Start_Current_Control_flag = false;
        OrderPackets::Start_Speed_Control_flag = false;
        OrderPackets::Change_angle_offset_flag = false;
    }
    inline static bool is_connected() {
#ifdef STLIB_ETH
        return OrderPackets::control_station_tcp->is_connected();
#else
        return true;
#endif
    }
    inline static float duty_cycle_u_received{};
    inline static float duty_cycle_v_received{};
    inline static float duty_cycle_w_received{};
    inline static uint32_t modulation_frequency_received{};
    inline static int64_t dead_time_ns_received{};
    inline static uint32_t commutation_frequency_received{};
    inline static float voltage_reference_received{};
    inline static float voltage_max_received{};
    inline static float d_current_reference_received{};
    inline static float q_current_reference_received{};
    inline static float target_linear_speed_received{};
    inline static float angle_offset_received{};
};

} // namespace BCU
