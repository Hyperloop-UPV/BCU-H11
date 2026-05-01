#pragma once
#include "BCU/Topology/Topology.hpp"

namespace BCU {
class CurrentController {
public:
    static inline Types::DutyCycles execute(const Types::TelemetryData& data) {
// TODO: Integrate BOTH Speetecs
// get information from sensors
#if SPEETEC == 1
        float position = static_cast<float>(data.speetec1.position) / ControlConf::polar_pitch;
#elif SPEETEC == 2
        float position = static_cast<float>(data.speetec1.position) / ControlConf::polar_pitch;
#endif
#if INVERTER == 1
        float u_current = data.currentSenseA.U;
        float v_current = data.currentSenseA.V;
        float w_current = data.currentSenseA.W;
#elif INVERTER == 2
        float u_current = data.currentSenseBU;
        float v_current = data.currentSenseBV;
        float w_current = data.currentSenseBW;
#else
        float u_current = (data.currentSenseAU + data.currentSenseBU) / 2.0f;
        float v_current = (data.currentSenseAV + data.currentSenseB_V) / 2.0f;
        float w_current = (data.currentSenseAW + data.currentSenseBW) / 2.0f;
#endif
#if DCLINK == 1
        float dc_link = data.VoltageSense.A;
#elif DCLINK == 2
        float dc_link = data.VoltageSense.B;
#else
        float dc_link = (data.VoltageSense.A + data.VoltageSense.B) / 2.0f;
#endif
//TODO ELIMINATE FMOD AND FLOOR
        electrical_angle =
            fmod(M_PI * (position - (2.0f * floor(position / 2.0f))) + angle_offset, 2 * M_PI);

        float alpha, beta;

        clarke_transform(u_current, v_current, w_current, alpha, beta);
        park_transform(alpha, beta, electrical_angle, measured.d, measured.q);

        error.d = reference.d - measured.d;
        error.q = reference.q - measured.q;

        update_d(error.d);
        update_q(error.q);

        float alpha_ref, beta_ref, u_ref, v_ref, w_ref;
        inverse_park_transform(alpha_ref, beta_ref);
        inverse_clarke_transform(alpha_ref, beta_ref, u_ref, v_ref, w_ref);

        offset = (std::max({u_ref, v_ref, w_ref}) + std::min({u_ref, v_ref, w_ref})) / 2.0f;
        target.u = u_ref - offset;
        target.v = v_ref - offset;
        target.w = w_ref - offset;
        // positive between 0.0-100.0
        return Types::DutyCycles(
            100.0f * (u_ref / dc_link + 1.0f) / 2.0f,
            100.0f * (v_ref / dc_link + 1.0f) / 2.0f,
            100.0f * (w_ref / dc_link + 1.0f) / 2.0f
        );
    }
    static inline void reset() {
        d_current_control.reset();
        q_current_control.reset();
        output.d = 0.0f;
        output.q = 0.0f;
    }
    static inline void set_angle_offset(float angle) { angle_offset = angle; }
    static inline void set_d_ref(float d_ref) { reference.d = d_ref; }
    static inline void set_q_ref(float q_ref) { reference.q = q_ref; }

private:
    static inline void clarke_transform(float u, float v, float w, float& alpha, float& beta) {
        alpha = (2.0f * u - v - w) / 3.0f;
        beta = CLARKE_BETA_COEFF * (v - w); // CLARKE BETA COEF = 1/sqrt(3)
    }
    static inline void park_transform(float alpha, float beta, float theta, float& d, float& q) {
#if TRIGONOMETRICS_CALC == 1
        cos_tetha = std::cos(theta);
        sin_tetha = std::sin(theta);

#elif TRIGONOMETRICS_CALC == 2
        int theta_q31, cos_q31, sin_q31, cos_theta, sin_theta;
        theta_q31 = RotationComputer::radian_f32_to_q31(theta);
        RotationComputer::cos_and_sin(&theta_q31, &cos_q31, &sin_q31, 1);
        cos_theta = RotationComputer::q31_to_radian_f32(cos_q31);
        sin_theta = RotationComputer::q31_to_radian_f32(sin_q31);
#endif
        d = alpha * cos_tetha + beta * sin_tetha;
        q = -alpha * sin_tetha + beta * cos_tetha;
    }
    static inline void
    inverse_clarke_transform(float alpha, float beta, float& u, float& v, float& w) {
        u = alpha;
        v = (-alpha + M_SQRT3 * beta) / 2.0f;
        w = (-alpha - M_SQRT3 * beta) / 2.0f;
    }
    static inline void inverse_park_transform(float& alpha, float& beta) {
        alpha = output.d * cos_tetha - output.q * sin_tetha;
        beta = output.d * sin_tetha + output.q * cos_tetha;
    }
    static inline void update_d(float current_error) {
        d_current_control.input(current_error);
        d_current_control.execute();
        output.d = d_current_control.output_value;
        if (output.d > ControlConf::D_MAX_OUTPUT)
            output.d = ControlConf::D_MAX_OUTPUT;
        if (output.d < ControlConf::D_MIN_OUTPUT)
            output.d = ControlConf::D_MIN_OUTPUT;
    }
    static inline void update_q(float current_error) {
        q_current_control.input(current_error);
        q_current_control.execute();
        output.q = q_current_control.output_value;
        if (output.q > ControlConf::D_MAX_OUTPUT)
            output.q = ControlConf::D_MAX_OUTPUT;
        if (output.q < ControlConf::D_MIN_OUTPUT)
            output.q = ControlConf::D_MIN_OUTPUT;
    }
    inline static PI<IntegratorType::Trapezoidal, float> d_current_control{
        ControlConf::D_KP_Current,
        ControlConf::D_KI_Current,
        ControlConf::CurrentControlPeriod
    };
    inline static PI<IntegratorType::Trapezoidal, float> q_current_control{
        ControlConf::Q_KP_Current,
        ControlConf::Q_KI_Current,
        ControlConf::CurrentControlPeriod
    };
    struct Output {
        float d;
        float q;
        Output() : d(0.0f), q(0.0f) {}
    };
    struct Error {
        float d;
        float q;
        Error() : d(0.0f), q(0.0f) {}
    };
    struct Reference {
        float d;
        float q;
        Reference() : d(0.0f), q(0.0f) {}
    };
    struct Measured {
        float d;
        float q;
        Measured() : d(0.0f), q(0.0f) {}
    };
    inline static Reference reference{};
    inline static Error error{};
    inline static Output output{};
    inline static Measured measured{};
    inline static Types::DutyCycles target;
    inline static float cos_tetha{0.0f};
    inline static float sin_tetha{0.0f};
    inline static float electrical_angle{0.0f};
    inline static float angle_offset{0.0f};
    inline static float offset{0.0f};
    static constexpr float CLARKE_BETA_COEFF = 1.0f / M_SQRT3;

public:
    struct Data {
        Reference& reference;
        Error& error;
        Output& output;
        Measured measured;
        Types::DutyCycles& target;
        float& electrical_angle;
        float& angle_offset;
        Data(
            Reference& ref,
            Error& err,
            Output& out,
            Measured& measured,
            Types::DutyCycles& target,
            float& electrical_angle,
            float& angle
        )
            : reference(ref), error(err), output(out), measured(measured), target(target),
              electrical_angle(electrical_angle), angle_offset(angle) {}
    };
    inline static Data subscribe() {
        return Data(reference, error, output, measured, target, electrical_angle, angle_offset);
    }
};
} // namespace BCU
