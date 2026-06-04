#pragma once
#include "BCU/Data/Data.hpp"
#include "BCU/Comms/Comms.hpp"
#ifdef USE_MATLAB_FOC_SPEED
#include "BCU/FOC_MATLAB/Speed_Controller.h"
#endif
namespace BCU {
class SpeedController {
public:
    struct Data {
        float& target_speed;
        float& output_iq_ref;
        float& error_speed;
        Data(float& target, float& output_iq, float& err)
            : target_speed(target), output_iq_ref(output_iq), error_speed(err) {}
    };
    inline static Data subscribe() {
        return Data(target_linear_speed, output_iq_ref, linear_speed_error);
    }
    inline static float execute(const Types::TelemetryData& data) {
// Read speed from speetec
#if SPEETEC == 1
        float speed = data.speetec1.speed;
#elif SPEETEC == 2
        speed = data.speetec2.speed;
#endif

#if USE_MATLAB_FOC_SPEED == 1
        output_iq_ref = Matlab_Control.step(target_linear_speed, speed, linear_speed_error);
#elif
        linear_speed_error = abs(speed - target_linear_speed);
        speed_control.input(linear_speed_error);
        speed_control.execute();
        output_iq_ref = speed_control.output_value;
#endif
        return output_iq_ref;
    }
    inline static void reset() {
#if USE_MATLAB_FOC_SPEED == 1
        Matlab_Control.SpeedController_DW.speed_integrator_state = 0.0f;
#elif
        speed_control.reset();
#endif
        linear_speed_error = 0.0f;
    }
    inline static void set_speed_m_s(float speed) { target_linear_speed = speed; }

private:
    inline static PI<IntegratorType::Trapezoidal, float> speed_control{
        ControlConf::KP_Speed,
        ControlConf::KI_Speed,
        ControlConf::SpeedControlPeriod
    };
    inline static float target_linear_speed{0.0f};
    inline static float linear_speed_error{0.0f};
    inline static float saturated_pi_output{0.0f};
    inline static float output_iq_ref;
#if USE_MATLAB_FOC_SPEED == 1
    inline static MATLAB::SpeedControl Matlab_Control{};
#endif
};
} // namespace BCU
