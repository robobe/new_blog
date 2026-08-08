#include <spdlog/spdlog.h>

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

    spdlog::debug("Connecting to camera {}", 0);
    spdlog::info("Application started");
    spdlog::warn("This is a warning");
    spdlog::error("Example error code: {}", 42);
}
