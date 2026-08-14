#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

enum class FlightMode
{
    manual,
    altitude_hold,
    position_hold
};

[[nodiscard]] constexpr std::string_view mode_name(FlightMode mode)
{
    switch (mode)
    {
        case FlightMode::manual:
            return "manual";
        case FlightMode::altitude_hold:
            return "altitude hold";
        case FlightMode::position_hold:
            return "position hold";
    }
    return "unknown";
}

struct Position
{
    double north_m{};
    double east_m{};
    double altitude_m{};
};

class VehicleConfig
{
public:
    VehicleConfig(std::string name, FlightMode mode, double altitude_limit_m)
        : name_{std::move(name)}, mode_{mode}, altitude_limit_m_{altitude_limit_m}
    {
        if (name_.empty())
        {
            throw std::invalid_argument{"vehicle name must not be empty"};
        }
        if (!std::isfinite(altitude_limit_m_) || altitude_limit_m_ <= 0.0)
        {
            throw std::invalid_argument{"altitude limit must be positive and finite"};
        }
    }

    [[nodiscard]] std::string_view name() const noexcept { return name_; }
    [[nodiscard]] FlightMode mode() const noexcept { return mode_; }
    [[nodiscard]] double altitude_limit_m() const noexcept { return altitude_limit_m_; }

private:
    std::string name_;
    FlightMode mode_;
    double altitude_limit_m_;
};

int main()
{
    const VehicleConfig vehicle{"scout", FlightMode::altitude_hold, 120.0};
    const Position home{0.0, 0.0, 12.5};

    std::cout << "Vehicle: " << vehicle.name() << '\n';
    std::cout << "Mode: " << mode_name(vehicle.mode()) << '\n';
    std::cout << "Limit: " << vehicle.altitude_limit_m() << " m\n";
    std::cout << "Home altitude: " << home.altitude_m << " m\n";
}
