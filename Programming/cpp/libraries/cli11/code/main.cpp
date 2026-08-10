#include <CLI/CLI.hpp>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    CLI::App app{"A minimal CLI11 hello-world example"};

    std::string name = "world";
    app.add_option("-n,--name", name, "Name to greet");

    CLI11_PARSE(app, argc, argv);

    std::cout << "Hello, " << name << "!\n";
    return 0;
}
