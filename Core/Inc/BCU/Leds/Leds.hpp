#include "BCU/Topology/Topology.hpp"

namespace BCU {
class Leds {
public:
    Leds() = delete;
    template <typename type> static void turn_on() {
        auto& led = get_led<type>();
        led.turn_on();
    }
    template <typename type> static void turn_off() {
        auto& led = get_led<type>();
        led.turn_off();
    }

private:
    template <typename type> static constexpr auto& get_led() {
        if constexpr (std::is_same_v<type, Types::LedConnecting>)
            return connecting;
        else if constexpr (std::is_same_v<type, Types::LedOperational>)
            return operational;
        else if constexpr (std::is_same_v<type, Types::LedFault>)
            return fault;
        else if constexpr (std::is_same_v<type, Types::LedCan>)
            return can;
        else if constexpr (std::is_same_v<type, Types::LedAccelerating>)
            return accelerating;
        else if constexpr (std::is_same_v<type, Types::LedBraking>)
            return braking;
        else
            compile_error("That led doesn't exist");
    }
    static inline Types::LedConnecting connecting{};
    static inline Types::LedOperational operational{};
    static inline Types::LedFault fault{};
    static inline Types::LedCan can{};
    static inline Types::LedAccelerating accelerating{};
    static inline Types::LedBraking braking{};
};

} // namespace BCU
