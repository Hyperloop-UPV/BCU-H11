#pragma once
#include "ST-LIB.hpp"
/*
Data has to provide:
struct Data{
    float sensor_data[N];
};
N == Number of sensors declared.
the union is need to allow iterate easily over the sensor data

The order of the types must be the same as the order in Data.
*/

namespace SensorConfig{
     struct Config{
        float Slope;
        float Offset;
        consteval Config(float Slope = 1.0,float Offset = 0.0):
        Slope(Slope),Offset(Offset)
        {}
    };
    struct No_Supply{
        void turn_on(){}
        void turn_off(){}
        No_Supply(){}
    };
}
//In case of No power_supply, DO_Supply = No_Supply
template <typename Board,typename Data,typename DO_Supply,const auto& config,auto&... adcs>
class Sensor{
    using ConfigType = std::decay_t<decltype(config)>; // Decay limpia las referencias.
    static constexpr std::size_t NumberSensors = sizeof...(adcs);
    static_assert(NumberSensors > 0, "Must be at least one Sensor");
    static_assert(std::is_convertible_v<ConfigType,std::array<SensorConfig::Config,NumberSensors>>, "Must be convertible to std::array<SensorConfig::Config,NumberSensors>");
    static_assert(
        std::extent_v<decltype(Data{}.sensor_data)> == NumberSensors,
    "Data needs to have an array sensor_data with the same size as the number of sensors"
    );
    private:
    Data data{};
    std::array<LinearSensor<float>,NumberSensors> sensors;
    DO_Supply supply{};
    template<std::size_t... Is>
    Sensor(std::index_sequence<Is...>):
        sensors{LinearSensor<float>(Board::template instance_of<adcs>(),config[Is].Slope,config[Is].Offset,data.sensor_data[Is])...}
    {} 

    template<auto& TargetADC>
    static constexpr std::size_t get_index(){
        std::size_t index = 0;
        bool found = ((&TargetADC == &adcs ? true : (++index,false)) || ...);
        if(!found){
            return NumberSensors;
        }
        return index;
    }
    public:
    Sensor(): 
    Sensor(std::make_index_sequence<NumberSensors>{}){}

    const Data& subscribe(){
        return data;
    }
    template<auto& TargetADC>
    void set_offset(float offset){
        constexpr std::size_t index = get_index<TargetADC>();
        static_assert(index < NumberSensors,"The ADC is not part of this class sensor");
        sensors[index].set_offset(offset);
    }
    template<auto& TargetADC>
    void set_slope(float slope){
        constexpr std::size_t index = get_index<TargetADC>();
        static_assert(index < NumberSensors,"The ADC is not part of this class sensor");
        sensors[index].set_gain(slope);
    }
    void read(){
        for(auto& sensor : sensors){
            sensor.read();
        }
    }
};
