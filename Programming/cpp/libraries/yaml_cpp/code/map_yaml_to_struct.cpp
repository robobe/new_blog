#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>

// These structures mirror config.yaml.
struct AppConfig {
    std::string name;
    int port;
};

struct Config {
    AppConfig app;
    std::vector<std::string> features;
};

// Teach yaml-cpp how to convert YAML nodes into our structures.
namespace YAML {

template<>
struct convert<AppConfig> {
    static bool decode(const Node& node, AppConfig& value)
    {
        if (!node.IsMap() || !node["name"] || !node["port"]) {
            return false;
        }

        value.name = node["name"].as<std::string>();
        value.port = node["port"].as<int>();
        return true;
    }
};

template<>
struct convert<Config> {
    static bool decode(const Node& node, Config& value)
    {
        if (!node.IsMap() || !node["app"] || !node["features"]) {
            return false;
        }

        value.app = node["app"].as<AppConfig>();
        value.features = node["features"].as<std::vector<std::string>>();
        return true;
    }
};

} // namespace YAML

int main(int argc, char* argv[])
{
    const std::string input_path = argc > 1 ? argv[1] : "config.yaml";

    try {
        const Config config = YAML::LoadFile(input_path).as<Config>();

        std::cout << "Application: " << config.app.name << '\n';
        std::cout << "Port: " << config.app.port << '\n';
        std::cout << "Features:\n";
        for (const std::string& feature : config.features) {
            std::cout << "  - " << feature << '\n';
        }
    } catch (const YAML::Exception& error) {
        std::cerr << "YAML error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
