#include <iostream>
#include <string>
#include <string_view>
#include <utility>

enum class SensorState
{
    offline,
    ready,
    fault
};

[[nodiscard]] constexpr std::string_view state_name(SensorState state)
{
    // TODO: Return a useful name for every SensorState value.
    (void)state;
    return "unknown";
}

struct SensorConfig
{
    std::string name;
    double sample_rate_hz;

    SensorConfig(std::string sensor_name, double rate_hz)
        : name{std::move(sensor_name)}, sample_rate_hz{rate_hz}
    {
        // TODO: Reject an empty name and a non-positive sample rate.
    }
};

int main()
{
    const SensorConfig sensor{"imu", 100.0};
    const SensorState state{SensorState::ready};
    std::cout << sensor.name << ' ' << state_name(state) << " at "
              << sensor.sample_rate_hz << " Hz\n";
}
