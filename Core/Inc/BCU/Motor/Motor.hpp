#pragma once
#include "../Traits/Traits.hpp"
#include "ST-LIB.hpp"

namespace Drivers{

template<typename Board, typename MotorTimer, typename PhaseR, typename PhaseS, typename PhaseT, typename BufferEnable>
class Motor {
public: 
    enum class State : uint8_t {
        Inactive,
        Active
    };
    struct Data {
        int frequency;
        float duty_cycle_r;
        float duty_cycle_s;
        float duty_cycle_t;
        State state;
    };

    explicit Motor(uint64_t deadtime_ns = 300) :
        motor_timer_(&Board::template instance_of<ktimer_motor>()),
        phase_r_(motor_timer_.template get_dual_pwm<kpinR_P, kpinR_N>()),
        phase_s_(motor_timer_.template get_dual_pwm<kpinS_P, kpinS_N>()),
        phase_t_(motor_timer_.template get_dual_pwm<kpinT_P, kpinT_N>())
    {
        set_dead_time(deadtime_ns);
    }

    const Data& subscribe() { return data_; }

    template<typename T>
    inline void set_duty_cycle(float duty_cycle) {
        static_assert(std::is_same_v<T, PhaseR> || std::is_same_v<T, PhaseS> || std::is_same_v<T, PhaseT>, "Invalid phase type");
        
        if constexpr (std::is_same_v<T, PhaseR>) {
            phase_r_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_r = phase_r_.get_duty_cycle();
        } else if constexpr (std::is_same_v<T, PhaseS>) {
            phase_s_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_s = phase_s_.get_duty_cycle();
        } else if constexpr (std::is_same_v<T, PhaseT>) {
            phase_t_.set_duty_cycle(duty_cycle);
            data_.duty_cycle_t = phase_t_.get_duty_cycle();
        }
    }

    inline void set_duty_cycle(float duty_cycle_R, float duty_cycle_S, float duty_cycle_T) {
        phase_r_.set_duty_cycle(duty_cycle_R);
        phase_s_.set_duty_cycle(duty_cycle_S);
        phase_t_.set_duty_cycle(duty_cycle_T);
        data_.duty_cycle_r = phase_r_.get_duty_cycle();
        data_.duty_cycle_s = phase_s_.get_duty_cycle();
        data_.duty_cycle_t = phase_t_.get_duty_cycle();
    }

    inline void set_frequency(uint32_t frequency) {
        motor_timer_.set_pwm_frequency(frequency);
        data_.frequency = motor_timer_.pwm_frequency;
    }

    inline void set_dead_time(int64_t dead_time_ns) {
        phase_r_.set_dead_time(dead_time_ns);
        phase_s_.set_dead_time(dead_time_ns);
        phase_t_.set_dead_time(dead_time_ns);
    }

    inline void turn_on() {
        buffer_enable_.turn_on();
        data_.state = State::Active;
    }

    inline void turn_off() {
        buffer_enable_.turn_off();
        set_duty_cycle(0.0f, 0.0f, 0.0f);
        data_.state = State::Inactive;
    }

private:
    // Constant hardware
    static constexpr auto& ktimer_motor = TimerTraits<MotorTimer>::dev;
    static constexpr auto& kpinR_P = PhaseTraits<PhaseR>::pin_p;
    static constexpr auto& kpinR_N = PhaseTraits<PhaseR>::pin_n;
    static constexpr auto& kpinS_P = PhaseTraits<PhaseS>::pin_p;
    static constexpr auto& kpinS_N = PhaseTraits<PhaseS>::pin_n;
    static constexpr auto& kpinT_P = PhaseTraits<PhaseT>::pin_p;
    static constexpr auto& kpinT_N = PhaseTraits<PhaseT>::pin_n;

    BufferEnable buffer_enable_{};
    static inline Data data_{};
    MotorTimer motor_timer_;
    PhaseR phase_r_;
    PhaseS phase_s_;
    PhaseT phase_t_;
};
}
