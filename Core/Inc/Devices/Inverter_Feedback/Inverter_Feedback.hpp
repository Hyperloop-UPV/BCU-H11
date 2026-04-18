#pragma once
#include "ST-LIB.hpp"

namespace Devices {
template <typename Board, typename Data, auto&... DInput> class Inverter_Feedback {
public:
    explicit Inverter_Feedback() {}
    const Data& subscribe() { return data_; }
    void read() { read(std::make_index_sequence<kDInputs_Total>{}); }

private:
    template <std::size_t... I> void read(std::index_sequence<I...>) {
        ((data_.raw[I] = Board::template instance_of<DInput>().read()), ...);
    }
    static constexpr std::size_t kDInputs_Total = sizeof...(DInput);
    static_assert(
        requires(Data d) { d.raw; } &&
            std::extent_v<decltype(std::declval<Data>().raw)> == kDInputs_Total,
        "The size of data must be the same as total Inputs"
    );
    static inline Data data_{};
};
} // namespace Devices
