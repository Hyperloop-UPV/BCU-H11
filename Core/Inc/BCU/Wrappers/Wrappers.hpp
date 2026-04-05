#pragma once
#include "ST-LIB.hpp"


namespace BCU {
template <typename Board,Types::OLogic Logic,const auto& DO>
class DigitalOutputWrapper {    
    private:
    using DigitalOutput = ST_LIB::DigitalOutputDomain::DigitalOutput;

    public:
    consteval DigitalOutputWrapper(){}

    inline void turn_on() {
        auto& gpio_instance = Board::template instance_of<DO>();
        if constexpr (Logic == Types::OLogic::N_OPEN) {
            gpio_instance.turn_on();
        } else {
            gpio_instance.turn_off();
        }
    }
    inline void turn_off() {
        auto& gpio_instance = Board::template instance_of<DO>();
        if constexpr (Logic == Types::OLogic::N_CLOSE) {
            gpio_instance.turn_off();
        } else {
            gpio_instance.turn_on();
        }
    }
};

} // namespace BCU