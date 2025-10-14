---
tags:
    - cpp
    - template
---


{{ page_folder_links() }}


```cpp
#include <iostream>

template <typename T>
T add(T a, T b) {
    return a + b;
}

int main() {
    std::cout << add(3, 4) << "\n";       // int
    std::cout << add(2.5, 1.2) << "\n";   // double
    std::cout << add(std::string("Hi "), std::string("there")) << "\n"; // string
}

```