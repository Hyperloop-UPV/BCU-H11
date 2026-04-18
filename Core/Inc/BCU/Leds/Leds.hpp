#include "BCU/Topology/Topology.hpp"

namespace BCU {
class Leds {
public:
    template <typename type> static void turn_on() {
        auto& led = get_led<type>();
        led.turn_on();
    }
    template <typename type> static void turn_off() {
        auto& led = get_led<type>();
        led.turn_off();
    }
    template <typename type> static void toggle() {
        auto& led = get_led<type>();
        led.toggle();
    }
    

private:
    Leds() = delete;
    template <typename type> static constexpr auto& get_led() {
        if constexpr (std::is_same_v<type, Types::LedConnecting>)
            return connecting;
        else if constexpr (std::is_same_v<type, Types::LedOperational>)
            return operational;
        else if constexpr (std::is_same_v<type, Types::LedFault>)
            return fault;
        else if constexpr (std::is_same_v<type, Types::LedSpaceVector>)
            return spaceVector;
        else if constexpr (std::is_same_v<type, Types::LedCurrentControl>)
            return currentControl;
        else if constexpr (std::is_same_v<type, Types::LedSpeedControl>)
            return speedControl;
        else
            compile_error("That led doesn't exist");
    }
    static inline Types::LedConnecting connecting{};
    static inline Types::LedOperational operational{};
    static inline Types::LedFault fault{};
    static inline Types::LedSpaceVector spaceVector{};
    static inline Types::LedCurrentControl currentControl{};
    static inline Types::LedSpeedControl speedControl{};
};

} // namespace BCU
