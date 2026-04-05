#pragma once

#include "../BCU_Topology/BCU_Topology.hpp"
#include "../Traits/Traits.hpp"
#include "ST-LIB.hpp"

template<typename Board,typename MotorTimer,typename PhaseR, typename PhaseS, typename PhaseT,typename BufferEnable,typename MotorData,auto ActiveState, auto InactiveState>
class Motor{
    private:
    BufferEnable buffer_enable{};
    static inline MotorData data{};
    MotorTimer Motor_timer;
    PhaseR phase_r;
    PhaseS phase_s;
    PhaseT phase_t;
    // extract pin positive and negative for each phase with traits (THIS OBJECTS WILL ONLY BE USED IN COMPILE TIME 
    // Zero Cost
    static constexpr auto& timer_motor = TimerTraits<MotorTimer>::dev;
    
    static constexpr auto& pinR_P = PhaseTraits<PhaseR>::pin_p;
    static constexpr auto& pinR_N = PhaseTraits<PhaseR>::pin_n;

    static constexpr auto& pinS_P = PhaseTraits<PhaseS>::pin_p;
    static constexpr auto& pinS_N = PhaseTraits<PhaseS>::pin_n;

    static constexpr auto& pinT_P = PhaseTraits<PhaseT>::pin_p;
    static constexpr auto& pinT_N = PhaseTraits<PhaseT>::pin_n;


    public:
    Motor(uint64_t deadtime_ns = 300):
        Motor_timer(&Board::template instance_of<timer_motor>()),
        phase_r(Motor_timer.template get_dual_pwm<pinR_P, pinR_N>()),
        phase_s(Motor_timer.template get_dual_pwm<pinS_P, pinS_N>()),
        phase_t(Motor_timer.template get_dual_pwm<pinT_P, pinT_N>())
    {
        set_dead_time(deadtime_ns);
    }
    const MotorData& subscribe(){
        return data;
    }
    template<typename T>
    inline void set_duty_cycle(float duty_cycle){
        static_assert(std::is_same_v<T,PhaseR> || std::is_same_v<T,PhaseS> || std::is_same_v<T,PhaseT>, "Invalid phase type");
        if constexpr (std::is_same_v<T,PhaseR>) {
            phase_r.set_duty_cycle(duty_cycle);
            data.duty_cycle_R = phase_r.get_duty_cycle();
        } else if constexpr (std::is_same_v<T,PhaseS>) {
            phase_s.set_duty_cycle(duty_cycle);
            data.duty_cycle_S = phase_s.get_duty_cycle();
        } else if constexpr (std::is_same_v<T,PhaseT>) {
            phase_t.set_duty_cycle(duty_cycle);
            data.duty_cycle_T = phase_t.get_duty_cycle();
        }
    }
    inline void set_duty_cycle(float duty_cycle_R, float duty_cycle_S, float duty_cycle_T){
        phase_r.set_duty_cycle(duty_cycle_R);
        phase_s.set_duty_cycle(duty_cycle_S);
        phase_t.set_duty_cycle(duty_cycle_T);
        data.duty_cycle_R = phase_r.get_duty_cycle();
        data.duty_cycle_S = phase_s.get_duty_cycle();
        data.duty_cycle_T = phase_t.get_duty_cycle();
    }
    inline void set_frequency(uint32_t frequency){
        Motor_timer.set_pwm_frequency(frequency);
        data.frequency = Motor_timer.pwm_frequency;
    }
    inline void set_dead_time(int64_t dead_time_ns){
        phase_r.set_dead_time(dead_time_ns);
        phase_s.set_dead_time(dead_time_ns);
        phase_t.set_dead_time(dead_time_ns);
    }
    inline void turn_on(){
        buffer_enable.turn_on();
        data.state = ActiveState;
    }
    inline void turn_off(){
        buffer_enable.turn_off();
        set_duty_cycle(0.0f, 0.0f, 0.0f);
        data.state = InactiveState;
    }
};
