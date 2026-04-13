#pragma once
#include "ST-LIB.hpp"
#include "BCU/Traits/Traits.hpp"

namespace SensorConfig {
struct Config {
    float slope;
    float offset;
    consteval Config(float slope = 1.0, float offset = 0.0) : slope(slope), offset(offset) {}
};
} // namespace SensorConfig

template <typename Board, typename Data, typename DOSupply, const auto& kConfig, auto&... adcs>
class Sensor {
public:
    using ConfigType = std::decay_t<decltype(kConfig)>;
    static constexpr std::size_t kNumberSensors = sizeof...(adcs);

    static_assert(kNumberSensors > 0, "Must be at least one Sensor");
    static_assert(
        std::is_convertible_v<ConfigType, std::array<SensorConfig::Config, kNumberSensors>>,
        "kConfig must be convertible to std::array<SensorConfig::Config, kNumberSensors>"
    );

    static_assert(
        requires(Data d) { d.raw; } &&
            std::extent_v<decltype(std::declval<Data>().raw)> == kNumberSensors,
        "Data needs to have an array 'raw' with the same size as the number of sensors"
    );

    explicit Sensor() : Sensor(std::make_index_sequence<kNumberSensors>{}) {}

    const Data& subscribe() const { return data_; }

    template <auto& TargetADC> void set_offset(float offset) {
        constexpr std::size_t index = get_index<TargetADC>();
        static_assert(index < kNumberSensors, "The ADC is not part of this class sensor");
        sensors_[index].set_offset(offset);
    }

    template <auto& TargetADC> void set_slope(float slope) {
        constexpr std::size_t index = get_index<TargetADC>();
        static_assert(index < kNumberSensors, "The ADC is not part of this class sensor");
        sensors_[index].set_gain(slope);
    }

    void read() {
        for (auto& sensor : sensors_) {
            sensor.read();
        }
    }

    void turn_on() { supply_.turn_on(); }

    void turn_off() { supply_.turn_off(); }

private:
    static inline Data data_{};
    std::array<LinearSensor<float>, kNumberSensors> sensors_;
    [[no_unique_address]] DOSupply supply_{};

    template <std::size_t... Is>
    explicit Sensor(std::index_sequence<Is...>)
        : sensors_{LinearSensor<float>(
              Board::template instance_of<adcs>(),
              kConfig[Is].slope,
              kConfig[Is].offset,
              data_.raw[Is]
          )...} {}

    template <auto& TargetADC> static constexpr std::size_t get_index() {
        std::size_t index = 0;
        bool found = ((&TargetADC == &adcs ? true : (++index, false)) || ...);
        return found ? index : kNumberSensors;
    }
};
