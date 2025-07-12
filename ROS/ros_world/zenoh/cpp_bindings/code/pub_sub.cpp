#include <iostream>
#include <thread>
#include "zenoh.hxx"

using namespace zenoh;
void task1()
{
    Config config = Config::create_default();
    auto session = Session::open(std::move(config));
    // auto publisher = session.declare_publisher(KeyExpr("demo/example/simple"));

    for (int i = 0; i < 5; ++i) {
        // Publish using session.put
        session.put(KeyExpr("demo/example/simple"), Bytes("Simple from session.put! i=" + std::to_string(i)));

        // Publish using publisher.put
        // publisher.put("Simple from publisher.put!");

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}



int main()
{
    std::thread t1(task1);
    Config config = Config::create_default();
    auto session = Session::open(std::move(config));
    auto subscriber = session.declare_subscriber(
        KeyExpr("demo/example/simple"),
        [](const Sample &sample)
        {
            std::cout << "Received: " << sample.get_payload().as_string() << std::endl;
        },
        closures::none);

    t1.join(); // Wait for task1 to finish

    std::cout << "Demo completed.\n";
    return 0;
}
