#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    const std::string input_path = argc > 1 ? argv[1] : "config.yaml";
    try {
        const YAML::Node config = YAML::LoadFile(input_path);

        const std::string name = config["app"]["name"].as<std::string>();
        const int port = config["app"]["port"].as<int>();

        std::cout << "Application: " << name << '\n';
        std::cout << "Port: " << port << '\n';
        std::cout << "Features:\n";
        for (const YAML::Node& feature : config["features"]) {
            std::cout << "  - " << feature.as<std::string>() << '\n';
        }

    } catch (const YAML::Exception& error) {
        std::cerr << "YAML error: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
