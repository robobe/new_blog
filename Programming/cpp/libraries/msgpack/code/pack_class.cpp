#include <iostream>
#include <msgpack.hpp>

class Point
{
public:
    int x;
    int y;

    MSGPACK_DEFINE(x, y);
};

int main()
{
    Point p{10, 20};
    msgpack::sbuffer buf;
    msgpack::pack(buf, p);

    auto handle = msgpack::unpack(buf.data(), buf.size());
    msgpack::object obj = handle.get();

    Point p2;
    obj.convert(p2);
    std::cout << "Point: (" << p2.x << ", " << p2.y << ")\n";
    return 0;
}
