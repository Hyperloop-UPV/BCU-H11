#pragma once
#include "Devices/Traits/Traits.hpp"
#include "ST-LIB.hpp"

namespace Devices {

namespace ThreePhaseMotorDefs {
enum class State : uint8_t { Inactive = 0, Active = 1 };
struct Data {
    int64_t dead_time_ns;
    float duty_cycle_u;
    float duty_cycle_v;
    float duty_cycle_w;
    uint32_t frequency;
    union{
        State state;
        uint8_t raw_state;
    };
};
} // namespace ThreePhaseMotorDefs
template <
    typename Board,
    typename MotorTimer,
    typename PhaseU,
    typename PhaseV,
    typename PhaseW,
    typename BufferEnable>
class ThreePhaseMotor {
public:
    explicit ThreePhaseMotor(uint64_t deadtime_ns = 300)
        : motor_timer_(&Board::template instance_of<ktimer_motor>()),
          phase_u_(motor_timer_.template get_dual_pwm<kpinU_P, kpinU_N>()),
          phase_v_(motor_timer_.template get_dual_pwm<kpinV_P, kpinV_N>()),
          phase_w_(motor_timer_.template get_dual_pwm<kpinW_P, kpinW_N>()) {
        set_dead_time(deadtime_ns);
    }

    const ThreePhaseMotorDefs::Data& subscribe() { return data_; }

    template <typename T> inline void set_duty_cycle(float duty_cycle) {
        static_assert(
            std::is_same_v<T, PhaseU> || std::is_same_v<T, PhaseV> || std::is_same_v<T, PhaseW>,
            "Invalid phase type"
        );

        if constexpr (std::is_same_v<T, PhaseU>) {
            phase_u_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_u = phase_u_.get_duty_cycle();
        } else if constexpr (std::is_same_v<T, PhaseV>) {
            phase_v_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_v = phase_v_.get_duty_cycle();
        } else if constexpr (std::is_same_v<T, PhaseW>) {
            phase_w_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_w = phase_w_.get_duty_cycle();
        }
    }

    inline void set_duty_cycle(float duty_cycle_U, float duty_cycle_V, float duty_cycle_W) {
        phase_u_.set_duty_cycle(duty_cycle_U);
        phase_v_.set_duty_cycle(duty_cycle_V);
        phase_w_.set_duty_cycle(duty_cycle_W);
        data_.duty_cycle_u = phase_u_.get_duty_cycle();
        data_.duty_cycle_v = phase_v_.get_duty_cycle();
        data_.duty_cycle_w = phase_w_.get_duty_cycle();
    }

    inline void set_frequency(uint32_t frequency) {
        motor_timer_.set_pwm_frequency(frequency);
        data_.frequency = motor_timer_.pwm_frequency;
    }

    inline void set_dead_time(int64_t dead_time_ns) {
        phase_u_.set_dead_time(dead_time_ns);
        phase_v_.set_dead_time(dead_time_ns);
        phase_w_.set_dead_time(dead_time_ns);
        data_.dead_time_ns = dead_time_ns;
    }
    inline void setup_signals(
        float duty_cycle_U,
        float duty_cycle_V,
        float duty_cycle_W,
        uint32_t frequency,
        int64_t dead_time_ns
    ) {
        set_dead_time(dead_time_ns);
        set_frequency(frequency);
        set_duty_cycle(duty_cycle_U, duty_cycle_V, duty_cycle_W);
    }
    inline void engage() {
        buffer_enable_.turn_on();
        data_.state = ThreePhaseMotorDefs::State::Active;
    }
    inline void disengage() {
        buffer_enable_.turn_off();
        data_.state = ThreePhaseMotorDefs::State::Inactive;
    }
    inline void stop() {
        disengage();
        set_duty_cycle(0.0f, 0.0f, 0.0f);
    }
    inline void start(
        float duty_u,
        float duty_v,
        float duty_w,
        uint32_t frequency,
        int64_t dead_time_ns = 300
    ) {
        setup_signals(duty_u, duty_v, duty_w, frequency, dead_time_ns);
        engage();
    }

private:
    // Constant hardware
    static constexpr auto& ktimer_motor = TimerTraits<MotorTimer>::dev;
    static constexpr auto& kpinU_P = PhaseTraits<PhaseU>::pin_p;
    static constexpr auto& kpinU_N = PhaseTraits<PhaseU>::pin_n;
    static constexpr auto& kpinV_P = PhaseTraits<PhaseV>::pin_p;
    static constexpr auto& kpinV_N = PhaseTraits<PhaseV>::pin_n;
    static constexpr auto& kpinW_P = PhaseTraits<PhaseW>::pin_p;
    static constexpr auto& kpinW_N = PhaseTraits<PhaseW>::pin_n;

    [[no_unique_address]] BufferEnable buffer_enable_{};
    static inline ThreePhaseMotorDefs::Data data_{};
    MotorTimer motor_timer_;
    PhaseU phase_u_;
    PhaseV phase_v_;
    PhaseW phase_w_;
};
} // namespace Devices
