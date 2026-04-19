#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"
#include "BCU/Data/Data.hpp"
class Comms {
public:
    template <
        typename DataMotor,
        typename DataCurrentSenseA,
        typename DataCurrentSenseB,
        typename DataSpeetec>
    static void init(
        DataMotor& dataMotor,
        DataCurrentSenseA& dataCurrentSenseA,
        DataCurrentSenseB& dataCurrentSenseB,
        DataSpeetec& dataSpeetec
    ) {
        DataPackets::Motor_Info_init(
            dataMotor.duty_cycle_u,
            dataMotor.duty_cycle_v,
            dataMotor.duty_cycle_w,
            dataMotor.frequency,
            dataMotor.dead_time_ns,
            reinterpret_cast<DataPackets::Motor_State&>(dataMotor.state)
        );
        // DataPackets::State_Init();
        // DataPackets::Space_Vector_init();
        // DataPackets::DC_Link_init();
        // DataPackets::Motor_Currents_init();
        // DataPackets::SPEETEC_1_init();
        // DataPackets::Current_Control_init();
        // DataPackets::Speed_Control_init();
        // DataPackets::start();
        // OrderPackets::Stop_Control_init();
        // OrderPackets::Start_Test_PWM_init)();
        // OrderPackets::Configure_Modulation_Paremeters_init();
        // OrderPackets::Start_Space_Vector_init();
        // OrderPackets::Start_Speed_Control_init();
        // OrderPackets::Change_angle_offset_init();
        // OrderPackets::start();
    }
    inline static void clear_flags() {
        OrderPackets::Stop_Control_flag = false;
        OrderPackets::Start_Test_PWM_flag = false;
        OrderPackets::Configure_Modulation_Parameters_flag = false;
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
