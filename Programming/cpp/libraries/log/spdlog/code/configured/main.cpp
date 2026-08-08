#include <spdlog/cfg/env.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <array>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace
{
constexpr std::array<std::string_view, 3> component_names{
    "camera", "network", "control"};

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool known_component(const std::string &name)
{
    for (const auto component : component_names)
    {
        if (name == component)
        {
            return true;
        }
    }
    return false;
}

bool known_level(const std::string &level)
{
    static const std::set<std::string> levels{
        "trace", "debug", "info", "warn", "warning", "error", "err",
        "critical", "off"};
    return levels.count(level) != 0;
}

void validate_level_environment()
{
    const char *raw = std::getenv("SPDLOG_LEVEL");
    if (raw == nullptr || std::string_view(raw).empty())
    {
        return;
    }

    std::istringstream input(raw);
    std::string token;
    bool found_global = false;
    std::set<std::string> configured_components;

    while (std::getline(input, token, ','))
    {
        token = trim(token);
        if (token.empty())
        {
            throw std::runtime_error("SPDLOG_LEVEL contains an empty entry");
        }

        const auto separator = token.find('=');
        if (separator == std::string::npos)
        {
            if (found_global)
            {
                throw std::runtime_error(
                    "SPDLOG_LEVEL contains more than one global level");
            }
            if (!known_level(token))
            {
                throw std::runtime_error("unknown global log level: " + token);
            }
            found_global = true;
            continue;
        }

        const auto component = trim(token.substr(0, separator));
        const auto level = trim(token.substr(separator + 1));
        if (!known_component(component))
        {
            throw std::runtime_error("unknown logging component: " + component);
        }
        if (!known_level(level))
        {
            throw std::runtime_error(
                "unknown log level for " + component + ": " + level);
        }
        if (!configured_components.insert(component).second)
        {
            throw std::runtime_error(
                "duplicate SPDLOG_LEVEL component: " + component);
        }
    }
}

std::string environment_or(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    return value == nullptr || std::string_view(value).empty() ? fallback : value;
}

void validate_pattern(const std::string &pattern, const char *variable)
{
    constexpr std::string_view flags =
        "+nlLtvaAbhBcCYDxmdHIMSefFEprRTXzP^$@sg#!%uioO";

    for (std::size_t index = 0; index < pattern.size(); ++index)
    {
        if (pattern[index] != '%')
        {
            continue;
        }
        ++index;
        if (index == pattern.size())
        {
            throw std::runtime_error(std::string(variable) +
                                     " ends with an incomplete % flag");
        }
        if (pattern[index] == '-' || pattern[index] == '=')
        {
            ++index;
        }
        while (index < pattern.size() &&
               std::isdigit(static_cast<unsigned char>(pattern[index])))
        {
            ++index;
        }
        if (index < pattern.size() && pattern[index] == '!' && index > 0 &&
            std::isdigit(static_cast<unsigned char>(pattern[index - 1])))
        {
            ++index;
        }
        if (index == pattern.size() || flags.find(pattern[index]) == flags.npos)
        {
            const auto bad_flag = index == pattern.size()
                                      ? std::string("<missing>")
                                      : std::string(1, pattern[index]);
            throw std::runtime_error(std::string(variable) +
                                     " contains unknown flag %" + bad_flag);
        }
    }
}

std::string run_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y-%m-%d_%H-%M-%S");
    return output.str();
}

std::string csv_timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&time, &local);
    std::ostringstream output;
    output << std::put_time(&local, "%Y-%m-%dT%H:%M:%S");
    return output.str();
}

std::vector<spdlog::sink_ptr> make_diagnostic_sinks()
{
    const auto console_pattern = environment_or(
        "APP_CONSOLE_LOG_PATTERN", "[%H:%M:%S.%e] [%^%l%$] [%n] %v");
    const auto file_pattern = environment_or(
        "APP_FILE_LOG_PATTERN",
        "[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] [thread %t] %v");
    validate_pattern(console_pattern, "APP_CONSOLE_LOG_PATTERN");
    validate_pattern(file_pattern, "APP_FILE_LOG_PATTERN");

    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console->set_pattern(console_pattern);

    constexpr std::size_t five_megabytes = 5 * 1024 * 1024;
    auto file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        "logs/application.log", five_megabytes, 3);
    file->set_pattern(file_pattern);
    return {console, file};
}

void register_component_loggers(const std::vector<spdlog::sink_ptr> &sinks)
{
    for (const auto name : component_names)
    {
        auto logger = std::make_shared<spdlog::logger>(
            std::string(name), sinks.begin(), sinks.end());
        logger->set_level(spdlog::level::info);
        logger->flush_on(spdlog::level::warn);
        spdlog::register_logger(logger);
    }
}

std::shared_ptr<spdlog::logger> make_telemetry_logger()
{
    std::filesystem::create_directories("logs");
    const auto path = "logs/telemetry_" + run_timestamp() + ".csv";
    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path, true);
    sink->set_pattern("%v");
    auto logger = std::make_shared<spdlog::logger>("telemetry", sink);
    logger->info("timestamp,component,sequence,fps,latency_ms");
    return logger;
}
} // namespace

int main()
{
    try
    {
        validate_level_environment();
        const auto sinks = make_diagnostic_sinks();
        register_component_loggers(sinks);
        spdlog::cfg::load_env_levels();

        const auto camera = spdlog::get("camera");
        const auto network = spdlog::get("network");
        const auto control = spdlog::get("control");
        const auto telemetry = make_telemetry_logger();

        camera->debug("Opening camera {}", 0);
        camera->info("Camera stream started at {} FPS", 30);
        network->warn("Packet delay is {} ms", 18.4);
        control->info("Control loop ready");

        for (int sequence = 0; sequence < 3; ++sequence)
        {
            telemetry->info("{},{},{},{:.2f},{:.2f}", csv_timestamp(),
                            "camera", sequence, 29.97, 8.4 + sequence);
        }
        telemetry->flush();
        spdlog::shutdown();
    }
    catch (const std::exception &error)
    {
        std::cerr << "Logging configuration error: " << error.what() << '\n';
        spdlog::shutdown();
        return 1;
    }
}
