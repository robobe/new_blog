#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct AppConfig {
    std::string name;
    int port;
};

struct Config {
    AppConfig app;
    std::vector<std::string> features;
};

namespace YAML {

template<>
struct convert<AppConfig> {
    // Convert a C++ struct into a YAML node.
    static Node encode(const AppConfig& value)
    {
        Node node;
        node["name"] = value.name;
        node["port"] = value.port;
        return node;
    }

    // Convert a YAML node into a C++ struct.
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
    static Node encode(const Config& value)
    {
        Node node;
        node["app"] = value.app;
        node["features"] = value.features;
        return node;
    }

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
    const std::string output_path = argc > 2 ? argv[2] : "struct_updated.yaml";

    try {
        // 1. Load YAML into the structure.
        Config config = YAML::LoadFile(input_path).as<Config>();

        // 2. Update ordinary C++ members.
        config.app.name = "updated-yaml-demo";
        config.app.port = 9090;
        config.features.push_back("struct-mapping");

        // 3. Convert the structure back into YAML and save a new file.
        const YAML::Node output_node = YAML::convert<Config>::encode(config);
        std::ofstream output(output_path);
        if (!output) {
            std::cerr << "Cannot open output file: " << output_path << '\n';
            return 1;
        }

        output << output_node;
        std::cout << "Saved updated struct to " << output_path << '\n';
    } catch (const YAML::Exception& error) {
        std::cerr << "YAML error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
