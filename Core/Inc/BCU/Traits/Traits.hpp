#pragma once
#include "ST-LIB.hpp"
namespace Drivers{
    //  Trait for extracting timer reference from MotorTimer type 
    template<typename T> struct TimerTraits;

    template<const ST_LIB::TimerDomain::Timer& dev_ref>
    struct TimerTraits<ST_LIB::TimerWrapper<dev_ref>> {
        static constexpr const ST_LIB::TimerDomain::Timer& dev = dev_ref;
    };

    // Traits for extracting pin references from Phase types
    template<typename T>
    struct PhaseTraits;

    template<const ST_LIB::TimerDomain::Timer& dev, ST_LIB::TimerPin& P, ST_LIB::TimerPin& N>
    struct PhaseTraits<ST_LIB::DualPWM<dev, P, N>> {
        static constexpr const ST_LIB::TimerPin& pin_p = P;
        static constexpr const ST_LIB::TimerPin& pin_n = N;
    };

    struct No_Supply{
            void turn_on(){}
            void turn_off(){}
            No_Supply(){}
    };
}
