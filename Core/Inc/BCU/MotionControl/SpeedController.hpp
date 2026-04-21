#pragma once
#include "BCU/Data/Data.hpp"
#include "BCU/Comms/Comms.hpp"

namespace BCU{
class SpeedController{
    public:
    struct Data{
        float& target_speed;
        float& output_iq_ref;
        float& error_speed;
        Data(float& target,float& output_iq,float& err):target_speed(target),output_iq_ref(output_iq),error_speed(err){}
    };
    inline static Data subscribe(){
        return Data(target_linear_speed,output_iq_ref,linear_speed_error);
    }
    inline static float execute(Types::TelemetryData& data){
        #if SPEETEC == 1
        linear_speed_error = abs(data.speetec1.speed - target_linear_speed);
        #elif SPEETEC == 2
        linear_speed_error = abs(data.speetec2.speed - target_linear_speed);
        #endif
        speed_control.input(linear_speed_error);
        speed_control.execute();
        output_iq_ref = speed_control.output_value;
        return output_iq_ref;
    }
    inline static void reset(){
        speed_control.reset();
        linear_speed_error = 0.0f;
    }
    inline static void set_speed_m_s(float speed){
        target_linear_speed = speed;
    }
    private:
    
    inline static PI<IntegratorType::Trapezoidal,float> speed_control{ControlConf::KP_Speed,ControlConf::KI_Speed,ControlConf::SpeedControlPeriod};
    inline static float target_linear_speed{0.0f};
    inline static float linear_speed_error{0.0f};
    inline static float saturated_pi_output{0.0f};
    inline static float output_iq_ref;
};
}