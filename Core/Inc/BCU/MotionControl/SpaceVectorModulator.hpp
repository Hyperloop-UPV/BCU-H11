#pragma once
#include "BCU/Topology/Topology.hpp"
namespace BCU {
class SpaceVectorModulator {
public:
    struct Data {
        uint32_t& modulation_frequency;
        float& modulation_index;
        Data(uint32_t& freq, float& index) : modulation_frequency(freq), modulation_index(index) {}
    };
    inline static Data subscribe() { return Data(modulation_frequency, modulation_index); }
    inline static void set_modulation_index(float voltage_ref, float voltage_max) {
        if (voltage_max < 1.0f) {
            modulation_index = 0.0f;
            return;
        }
        if (voltage_ref < 0)
            voltage_ref = 0.0f;
        modulation_index = (voltage_ref * 2.0f) / voltage_max;
        if (modulation_index >= MAX_IMOD)
            modulation_index = MAX_IMOD;
    }
    inline static Types::DutyCycles execute() {
#if TRIGONOMETRICS_CALC == 1
        float sin_u = std::sin(TWO_PI * modulation_frequency * control_time) * modulation_index;
        float sin_v = std::sin(TWO_PI * modulation_frequency * control_time - phase_shift) * modulation_index;
        float sin_w = std::sin(TWO_PI * modulation_frequency * control_time + phase_shift) * modulation_index;
#elif TRIGONOMETRICS_CALC == 2
        int angle, sin_q31;
        angle = RotationComputer::radian_f32_to_q31(TWO_PI * modulation_frequency * control_time);
        RotationComputer::sin(&angle, &sin_q31, 1);
        float sin_u = RotationComputer::q31_to_radian_f32(sin_q31);
        angle = RotationComputer::radian_f32_to_q31(
            TWO_PI * modulation_frequency * control_time + phase_shift
        );
        RotationComputer::sin(&angle, &sin_q31, 1);
        float sin_v = RotationComputer::q31_to_radian_f32(sin_q31);
        angle = RotationComputer::radian_f32_to_q31(
            TWO_PI * modulation_frequency * control_time - phase_shift
        );
        RotationComputer::sin(&angle, &sin_q31, 1);
        float sin_w = RotationComputer::q31_to_radian_f32(sin_q31);
#endif
        // keep a control of the control_time value
        control_time += period_time;
        if (control_time >= 1.0) {
            control_time -= 1.0;
        }
        float offset = (std::max({sin_u, sin_v, sin_w}) + std::min({sin_u, sin_v, sin_w})) / 2.0f;
        sin_u = (sin_u - offset) * 100.0f;
        sin_v = (sin_v - offset) * 100.0f;
        sin_w = (sin_w - offset) * 100.0f;
        return Types::DutyCycles{sin_u, sin_v, sin_w};
    }
    inline static void set_modulation_freq(uint32_t freq) { modulation_frequency = freq; }

private:
    inline static float control_time = 0.0;
    inline static float modulation_index = 0.0;
    inline static uint32_t modulation_frequency = 0;
    inline static constexpr float MAX_IMOD = 2.0 / M_SQRT3;
    inline static constexpr float TWO_PI = 2.0 * M_PI;
    inline static constexpr float phase_shift = TWO_PI / 3.0f;
    inline static constexpr float period_time =
        (static_cast<float>(ControlConf::SpaceVectorPeriod) / 1'000'000);
};
} // namespace BCU
