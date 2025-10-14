#include <iostream>
#include <msgpack.hpp>
#include <vector>
#include <string>

int main() {
    std::cout << MSGPACK_VERSION << std::endl;

    // 1️⃣ Create some data
    std::vector<std::string> items = {"apple", "banana", "cherry"};

    // 2️⃣ Pack (serialize) data to binary buffer
    msgpack::sbuffer buffer; // simple buffer
    msgpack::pack(buffer, items);

    // 3️⃣ Unpack (deserialize) data
    msgpack::object_handle handle = msgpack::unpack(buffer.data(), buffer.size());
    msgpack::object deserialized = handle.get();

    // 4️⃣ Convert msgpack::object back to C++ type
    std::vector<std::string> unpacked;
    deserialized.convert(unpacked);

    // 5️⃣ Print the results
    std::cout << "Unpacked vector: ";
    for (auto &s : unpacked) std::cout << s << " ";
    std::cout << std::endl;

    return 0;
}