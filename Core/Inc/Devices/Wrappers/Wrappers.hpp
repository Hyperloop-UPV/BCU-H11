#pragma once
#include "ST-LIB.hpp"

namespace Devices {
// OUTPUT LOGIC (Normally Open or Normally Closed)
enum class OLogic : uint8_t {
    N_OPEN,
    N_CLOSE,
};

template <typename Board, OLogic Logic, const auto& DO> class DigitalOutputWrapper {
private:
    using DigitalOutput = ST_LIB::DigitalOutputDomain::DigitalOutput;

public:
    consteval DigitalOutputWrapper() {}

    inline void turn_on() {
        auto& gpio_instance = Board::template instance_of<DO>();
        if constexpr (Logic == OLogic::N_OPEN) {
            gpio_instance.turn_on();
        } else {
            gpio_instance.turn_off();
        }
    }
    inline void turn_off() {
        auto& gpio_instance = Board::template instance_of<DO>();
        if constexpr (Logic == OLogic::N_OPEN) {
            gpio_instance.turn_off();
        } else {
            gpio_instance.turn_on();
        }
    }
    inline void toggle(){
        auto& gpio_instance = Board::template instance_of<DO>();
        gpio_instance.toggle();
    }
};

} // namespace Devices
