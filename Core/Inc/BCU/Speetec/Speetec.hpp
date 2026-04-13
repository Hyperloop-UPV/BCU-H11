#pragma once
#include "ST-LIB.hpp"
#include "BCU/Traits/Traits.hpp"
namespace Devices {
template <
    typename Board,
    typename EncoderTimer,
    std::size_t Samples,
    typename EncoderSupply = Devices::NoSupply>
class Speetec {
public:
    struct Data {
        double position;
        double speed;
        double acceleration;
        ST_LIB::Direction direction;
    };
    explicit Speetec(double counter_distance_m, double sample_time_s)
        : encoder_timer_(&Board::template instance_of<ktimer_encoder>()),
          encoder_(encoder_timer_.template get_encoder()),
          encoderSensor_(ST_LIB::EncoderSensor<EncoderType, Samples>(
              encoder_,
              counter_distance_m,
              sample_time_s,
              &data_.direction,
              &data_.position,
              &data_.speed,
              &data_.acceleration
          )) {}
    const Data& inscribe() { return data_; }
    void turn_on() {
        supply_.turn_on();
        encoderSensor_.turn_on();
    }
    void turn_off() {
        supply_.turn_off();
        encoderSensor_.turn_off();
    }
    void read() { encoderSensor_.read(); }
    void reset() { encoderSensor_.reset(); }

private:
    using EncoderType = decltype(std::declval<EncoderTimer>().get_encoder());
    static constexpr auto& ktimer_encoder = TimerTraits<EncoderTimer>::dev;
    static inline Data data_{};
    EncoderTimer encoder_timer_;
    EncoderType encoder_;
    ST_LIB::EncoderSensor<EncoderType, Samples> encoderSensor_;
    [[no_unique_address]] EncoderSupply supply_{};
};
} // namespace Devices
