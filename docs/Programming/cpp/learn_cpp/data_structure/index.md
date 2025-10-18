---
title: Cpp data structure
tags:
    - cpp
    - data structure
---


{{ page_folder_links() }}

## STL Data structure

| Category                   | STL Container                                            | Description                                          |
| -------------------------- | -------------------------------------------------------- | ---------------------------------------------------- |
| **Sequence containers**    | [`std::vector`](#stdvector), [`std::array`](#stdarray), [`std::deque`](#stddeque), [`std::list`](#stdlist)   | Store ordered collections of elements                |
| **Associative containers** | `std::set`, `std::map`, `std::multiset`, `std::multimap` | Sorted data based on keys (balanced BST)             |
| **Unordered containers**   | `std::unordered_map`, `std::unordered_set`               | Hash-table-based lookup                              |
| **Adapters**               | `std::stack`, `std::queue`, `std::priority_queue`        | Provide restricted interfaces for specific use cases |
| | [std::tuple](#stdtuple) | hold different types of values together in a single object. |
| | [std::pair](#stdpair) | hold different types of values together in a single object. |

---

### Array

```cpp
#include <iostream>
    int main() {
    int arr[5];                // declare array of 5 ints (uninitialized)
    int nums[5] = {1, 2, 3};   // remaining elements default to 0 → {1,2,3,0,0}
    int more[] = {10, 20, 30}; // size inferred automatically (3)

    // Access elements
    std::cout << "First: " << arr[0] << std::endl;
    
    std::cout << "Last: " << more[4] << std::endl;

    nums[4] = 99;
    // Iterate
    for (int i = 0; i < 5; i++)
        std::cout << nums[i] << " ";
}
```

👉 [Run this code on Wandbox](https://wandbox.org/permlink/i9sMTYJtXlrs64Rc)

#### Dynamic array

```cpp
#include <iostream>
#include <memory>

int main() {
    // Create dynamic array managed by unique_ptr
    std::unique_ptr<int[]> arr = std::make_unique<int[]>(5);

    for (int i = 0; i < 5; ++i)
        arr[i] = i * 10;

    for (int i = 0; i < 5; ++i)
        std::cout << arr[i] << " ";
    std::cout << std::endl;

    // No need to delete[] manually — it’s automatic!
}
```
[Run](https://www.programiz.com/online-compiler/60mkWoyoIUwYH)

---

### std::array
Like array just better

```cpp
#include <array>
#include <iostream>

int main()
{
    std::array<int, 5> arr = {1, 2, 3, 4, 5};

    std::cout << arr.size();  // ✅ works
    std::cout << arr.front(); // ✅ first element
    std::cout << arr.back();  // ✅ last element

    // Range-based loop
    for (int val : arr)
        std::cout << val << " ";
}
```

[Run](https://www.programiz.com/online-compiler/4CghPhjTQpmeG)

---

### std::vector
std::vector is a dynamic array in C++ — it behaves like an array, but can automatically grow or shrink in size.

```cpp
#include <iostream>
#include <vector>

int main() {
    std::vector<int> nums = {10, 20, 30};
    nums.push_back(40); // Add element at the end
    nums.push_back(50);
    nums.pop_back(); // removes the last element
    nums.erase(nums.begin()); //removes to first element
    nums.insert(nums.begin()+2, 99);//insert new element before
    std::cout << "Vector elements: ";
    for (int n : nums)
        std::cout << n << " ";

    std::cout << "\nSize: " << nums.size() << std::endl;
    std::cout << "\nSize: " << nums.at(1) << std::endl;

}
```

---

### std::list

std::list is a doubly linked list container in the C++ Standard Library.

| Feature                       | `std::list`                     | `std::vector`                       |
| ----------------------------- | ------------------------------- | ----------------------------------- |
| **Underlying structure**      | Doubly linked list              | Dynamic contiguous array            |
| **Memory layout**             | Elements scattered across heap  | Elements stored contiguously        |
| **Random access (`list[i]`)** | ❌ Not supported                 | ✅ Supported (`O(1)`)                |
| **Insert/erase in middle**    | ✅ Fast (`O(1)`)                 | ⚠️ Slow (`O(n)`, shifting required) |
| **Iteration speed**           | ⚠️ Slower (poor cache locality) | ✅ Very fast (contiguous memory)     |
| **Reallocation**              | Never (nodes independent)       | May reallocate as it grows          |
| **Sorting**                   | Has built-in `list.sort()`      | Must use `std::sort()`              |
| **When to use**               | Frequent insert/erase in middle | Frequent access or random indexing  |

---

### std::deque

allows fast insertion and removal from both the front and the back.


```cpp title="with helper function to limit deque size"
#include <iostream>
#include <deque>

int main() {
    const size_t MAX_SIZE = 5;
    std::deque<int> dq;

    for (int i = 1; i <= 10; ++i) {
        if (dq.size() == MAX_SIZE)
            dq.pop_front();  // remove oldest when full

        dq.push_back(i);     // add new element

        std::cout << "Deque: ";
        for (int x : dq) std::cout << x << " ";
        std::cout << "\n";
    }
}
```

---

### std::tuple

```cpp
#include <tuple>
#include <iostream>
#include <string>

int main() {
    std::tuple<int, std::string, float> person(29, "Alice", 55.5f);

    std::cout << std::get<0>(person) << "\n"; // 29
    std::cout << std::get<1>(person) << "\n"; // Alice
    std::cout << std::get<2>(person) << "\n"; // 55.5
}

```


!!! tip ""
    ```
    Use auto [x, y, z] = func(); (C++17+) to unpack neatly, just like Python.
    ```
     

```cpp
#include <tuple>
#include <iostream>

std::tuple<double, double> demo() {
    return {1.0, 2.0};
}

int main() {
    auto [area, perimeter] = demo();
    std::cout << "Area = " << area << ", Perimeter = " << perimeter << "\n";
}


```

---

### std::pair
std::pair is a container that holds exactly two values, possibly of different types.

```cpp
std::pair<int, double> data(1, 2.5);
std::cout << data.first << ", " << data.second << "\n";

auto [a, b] = data;

```