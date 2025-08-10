---
title: CPP OOP class property 
tags:
    - cpp
    - oop
    - getter
    - setter
    - property
---

{{ page_folder_links() }}


```cpp
const std::string& getName() const { return name_; }
void setName(std::string name) { name_ = std::move(name); }

```

```cpp
struct Point {
private:
    int x_ = 0;
public:
    inline int x() const { return x_; }
    inline void x(int value) { x_ = value; }
};

```

```cpp
```