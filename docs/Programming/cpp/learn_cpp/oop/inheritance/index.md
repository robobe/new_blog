---
title: Cpp inheritance
tags:
    - cpp
    - oop
    - inheritance
    - class
---

# C++ inheritance

Inheritance lets one class reuse and extend another class.

The original class is called the **base class**.

The new class is called the **derived class**.

```text
Animal      <- base class
  |
  v
Dog         <- derived class
```

This post uses only simple single inheritance.

Multiple inheritance and virtual functions are separate topics.

---

## Base class

A base class contains data and functions that can be shared.

```cpp
#include <iostream>
#include <string>

class Animal {
public:
    void set_name(const std::string& name)
    {
        name_ = name;
    }

    void print_name() const
    {
        std::cout << name_ << "\n";
    }

private:
    std::string name_;
};
```

`Animal` is a normal class. It does not know anything about `Dog`.

---

## Derived class

Use `: public BaseClass` to inherit.

```cpp
#include <iostream>
#include <string>

class Animal {
public:
    void set_name(const std::string& name)
    {
        name_ = name;
    }

    void print_name() const
    {
        std::cout << name_ << "\n";
    }

private:
    std::string name_;
};

class Dog : public Animal {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog dog;

    dog.set_name("Rex"); // inherited from Animal
    dog.print_name();    // inherited from Animal
    dog.bark();          // defined in Dog
}
```

`Dog` can use the public functions from `Animal`.

`Dog` also adds its own function: `bark()`.

---

## What public inheritance means

```cpp
class Dog : public Animal {
};
```

This means:

> Dog is an Animal.

So a `Dog` object has the public interface of `Animal`.

```cpp
Dog dog;
dog.print_name();
```

Use public inheritance when the relationship is really **is-a**.

Good examples:

- `Dog` is an `Animal`
- `Car` is a `Vehicle`
- `Circle` is a `Shape`

Bad example:

- `Engine` is not a `Car`

For that case, use composition:

```cpp
class Car {
private:
    Engine engine_;
};
```

---

## Public, protected, and private inheritance

You can replace `public` with `protected` or `private`.

```cpp
class Dog : public Animal {
};

class Dog : protected Animal {
};

class Dog : private Animal {
};
```

This changes how the base class interface is exposed through the derived class.

| Inheritance | Base `public` members become | Common meaning |
| ----------- | ---------------------------- | -------------- |
| `public Animal` | `public` in `Dog` | `Dog` is an `Animal` |
| `protected Animal` | `protected` in `Dog` | only `Dog` and derived classes can use the `Animal` interface |
| `private Animal` | `private` in `Dog` | `Dog` uses `Animal` internally |

Most of the time, use public inheritance.

Example with public inheritance:

```cpp
class Animal {
public:
    void eat()
    {
    }
};

class Dog : public Animal {
};

int main()
{
    Dog dog;
    dog.eat(); // OK
}
```

Example with private inheritance:

```cpp
class Animal {
public:
    void eat()
    {
    }
};

class Dog : private Animal {
};

int main()
{
    Dog dog;
    // dog.eat(); // error: eat is private inside Dog
}
```

Simple rule:

- use `public` inheritance for normal **is-a** relationships
- use `private` inheritance rarely, when a class is implemented using another class
- use `protected` inheritance very rarely

---

## Private data is still private

The derived class does not get direct access to private members of the base
class.

```cpp
class Animal {
private:
    std::string name_;
};

class Dog : public Animal {
public:
    void bad()
    {
        // name_ = "Rex"; // error: name_ is private in Animal
    }
};
```

Private means:

> only the class itself can access it directly.

The derived class should use public or protected functions from the base class.

---

## Protected members

`protected` means:

- the base class can access it
- derived classes can access it
- outside code cannot access it directly

```cpp
#include <iostream>
#include <string>

class Animal {
protected:
    std::string name_;
};

class Dog : public Animal {
public:
    void set_dog_name(const std::string& name)
    {
        name_ = name; // OK: name_ is protected
    }

    void print() const
    {
        std::cout << name_ << "\n";
    }
};

int main()
{
    Dog dog;
    dog.set_dog_name("Rex");
    dog.print();

    // dog.name_ = "Max"; // error: outside code cannot access protected
}
```

Use `protected` carefully. In many cases, private data with public/protected
functions is easier to control.

---

## Constructor chaining

When you create a derived object, C++ constructs the base part first.

Then it constructs the derived part.

```text
create Dog
  |
  v
Animal constructor runs first
  |
  v
Dog constructor runs second
```

Example:

```cpp
#include <iostream>
#include <string>

class Animal {
public:
    Animal(const std::string& name)
        : name_(name)
    {
        std::cout << "Animal constructor: " << name_ << "\n";
    }

    void print_name() const
    {
        std::cout << name_ << "\n";
    }

private:
    std::string name_;
};

class Dog : public Animal {
public:
    Dog(const std::string& name, int age)
        : Animal(name), age_(age)
    {
        std::cout << "Dog constructor: age " << age_ << "\n";
    }

    void print_age() const
    {
        std::cout << age_ << "\n";
    }

private:
    int age_;
};

int main()
{
    Dog dog("Rex", 4);
    dog.print_name();
    dog.print_age();
}
```

Output:

```text
Animal constructor: Rex
Dog constructor: age 4
Rex
4
```

This line calls the base class constructor:

```cpp
: Animal(name), age_(age)
```

Read it like this:

- build the `Animal` part using `name`
- build the `Dog` part using `age`

---

## Default base constructor

If the base class has a default constructor, you do not have to call it
explicitly.

C++ calls it automatically before the derived constructor body runs.

```cpp
#include <iostream>

class Animal {
public:
    Animal()
    {
        std::cout << "Animal constructor\n";
    }
};

class Dog : public Animal {
public:
    Dog()
    {
        std::cout << "Dog constructor\n";
    }
};

int main()
{
    Dog dog;
}
```

Output:

```text
Animal constructor
Dog constructor
```

This constructor:

```cpp
Dog()
{
}
```

is the same as writing:

```cpp
Dog()
    : Animal()
{
}
```

You need to call the base constructor explicitly when the base constructor needs
arguments.

```cpp
class Animal {
public:
    Animal(const std::string& name)
    {
    }
};

class Dog : public Animal {
public:
    Dog()
        : Animal("Rex") // required
    {
    }
};
```

---

## Destructor order

Destruction happens in the opposite order.

```text
destroy Dog
  |
  v
Dog destructor runs first
  |
  v
Animal destructor runs second
```

Simple demo:

```cpp
#include <iostream>

class Animal {
public:
    Animal()
    {
        std::cout << "Animal constructor\n";
    }

    ~Animal()
    {
        std::cout << "Animal destructor\n";
    }
};

class Dog : public Animal {
public:
    Dog()
    {
        std::cout << "Dog constructor\n";
    }

    ~Dog()
    {
        std::cout << "Dog destructor\n";
    }
};

int main()
{
    Dog dog;
}
```

Output:

```text
Animal constructor
Dog constructor
Dog destructor
Animal destructor
```

---

## Base class interface, derived class object

A derived object can be used through the base class public interface.

```cpp
class Animal {
public:
    void eat() const
    {
        std::cout << "eat\n";
    }
};

class Dog : public Animal {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog dog;

    dog.eat();  // from Animal
    dog.bark(); // from Dog
}
```

The object is still a `Dog`, but it contains an `Animal` part.

```text
Dog object
+----------------+
| Animal part    |
| Dog part       |
+----------------+
```

You can also create a `Dog`, but use it through the `Animal` interface.

```cpp
#include <iostream>

class Animal {
public:
    void eat() const
    {
        std::cout << "eat\n";
    }
};

class Dog : public Animal {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog dog;

    Animal& animal_ref = dog;
    animal_ref.eat();  // OK: Animal interface
    // animal_ref.bark(); // error: Animal interface does not have bark()

    Animal* animal_ptr = &dog;
    animal_ptr->eat(); // OK
    // animal_ptr->bark(); // error
}
```

The real object is still a `Dog`.

But when you access it through `Animal&` or `Animal*`, C++ only lets you use the
functions that exist in the `Animal` interface.

### Heap allocation

You can also create the `Dog` on the heap and store the address in an `Animal*`.

```cpp
#include <iostream>

class Animal {
public:
    void eat() const
    {
        std::cout << "eat\n";
    }
};

class Dog : public Animal {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    Dog* dog = new Dog();

    Animal* animal = dog;
    animal->eat();  // OK: Animal interface
    // animal->bark(); // error: Animal interface does not have bark()

    delete dog;
}
```

The object is allocated as a `Dog`.

The pointer `animal` only exposes the `Animal` interface.

For modern C++, prefer smart pointers:

```cpp
#include <iostream>
#include <memory>

class Animal {
public:
    void eat() const
    {
        std::cout << "eat\n";
    }
};

class Dog : public Animal {
public:
    void bark() const
    {
        std::cout << "woof\n";
    }
};

int main()
{
    auto dog = std::make_unique<Dog>();

    Animal* animal = dog.get();
    animal->eat();
}
```

The `std::unique_ptr<Dog>` deletes the `Dog` automatically when it goes out of
scope.

Virtual functions change what happens when calling overridden functions through a
base reference or pointer. Deleting derived objects through base pointers is also
connected to virtual destructors. Those are separate topics.

---

## Quick rules

| Situation | Use |
| --------- | --- |
| A class is a more specific version of another class | public inheritance |
| A class only owns another object | composition |
| Derived class needs to initialize base data | call base constructor in initializer list |
| Base data should not be directly touched | keep it private |
| Derived class needs limited access | use protected carefully |

---

## What is not covered here

This post does not cover virtual functions.

Without virtual functions, inheritance is mostly about:

- reusing code
- sharing a base interface
- constructing base and derived parts correctly

Virtual functions are needed for runtime polymorphism. That should be learned in
a separate step.

- [Virtual functions](virtual_function/)

C++ also does not have a special `interface` keyword like Java or C#.

In C++, an interface is usually written as an abstract class with pure virtual
functions.

Small preview:

```cpp
class IAnimal {
public:
    virtual ~IAnimal() = default;
    virtual void speak() const = 0;
};
```

The `= 0` means the function must be implemented by a derived class.

Interfaces and abstract classes should be learned together in a separate post.
