---
title: Raw Pointers and References
tags:
    - cpp
    - ownership
    - pointers
    - references
---
# Lesson 2: Raw Pointers and References

Raw pointers and references usually **observe** objects that somebody else
owns. They provide access, but they do not automatically control lifetime.

## Learning goal

After this lesson, you should be able to:

- distinguish an owner from an observer;
- use a reference for required, non-owning access;
- use a pointer for optional, non-owning access;
- safely check and dereference a pointer;
- explain why an observer must not destroy an object.

## Start with a simple example

```cpp
#include <iostream>
#include <string>

struct Sensor
{
    std::string name;
    double value;
};

void print_sensor(const Sensor& sensor)
{
    std::cout << sensor.name << ": " << sensor.value << '\n';
}

int main()
{
    Sensor temperature{"temperature", 23.5}; // Owns the Sensor object.
    print_sensor(temperature);                // Observes it temporarily.
}
```

`temperature` is the owner because its scope controls the object's lifetime.
The parameter `sensor` is a non-owning reference. `print_sensor` may read the
object, but it does not keep or destroy it.

```text
Owner:    temperature
               │
               └── observed by → sensor
```

## Owner versus observer

An **owner** is responsible for ensuring that a resource is released. An
**observer** can access the resource while it is alive but does not release it.

For an ordinary local object:

```cpp
Sensor temperature{"temperature", 23.5};
Sensor& observer = temperature;
```

- `temperature` owns its own lifetime.
- `observer` is another name for the same object.
- Destroying `observer` does not destroy `temperature`.
- `observer` is only safe while `temperature` remains alive.

Raw pointers can technically own dynamically allocated objects, but modern C++
normally represents that ownership with a value or smart pointer. In this
course, assume `T*` is a non-owning observer unless an API explicitly documents
otherwise.

## References: required observers

A reference must refer to an object when it is created:

```cpp
Sensor temperature{"temperature", 23.5};
Sensor& sensor = temperature;

sensor.value = 24.0; // Modifies temperature.
```

Use a reference parameter when the object is required:

```cpp
void calibrate(Sensor& sensor)
{
    sensor.value += 0.5;
}
```

Use a `const` reference when the function only reads:

```cpp
void print_sensor(const Sensor& sensor)
{
    std::cout << sensor.value << '\n';
}
```

The reference does not make a copy and does not own the argument.

## Pointers: optional observers

A pointer can hold an object's address or `nullptr`:

```cpp
Sensor temperature{"temperature", 23.5};

Sensor* selected = &temperature; // Address of temperature.
Sensor* missing = nullptr;       // Points to no object.
```

Use a pointer parameter when no object is a valid case:

```cpp
void print_selected(const Sensor* sensor)
{
    if (sensor == nullptr)
    {
        std::cout << "No sensor selected\n";
        return;
    }

    std::cout << sensor->name << ": " << sensor->value << '\n';
}
```

The caller makes the optional case visible:

```cpp
print_selected(&temperature);
print_selected(nullptr);
```

`const Sensor*` means the function cannot modify the `Sensor` through this
pointer. It is still a non-owning observer.

## Pointer syntax

| Syntax | Meaning |
| --- | --- |
| `&object` | Get the object's address. |
| `*pointer` | Access the pointed-to object. |
| `pointer->member` | Access a member through a pointer. |
| `nullptr` | Point to no object. |

These two member accesses are equivalent:

```cpp
selected->value = 25.0;
(*selected).value = 25.0;
```

Check that an optional pointer is not null before dereferencing it.

## Choosing a parameter

| Requirement | Prefer | Ownership meaning |
| --- | --- | --- |
| Function needs its own independent object | `T` | The parameter owns its value. |
| Object is required and may be modified | `T&` | Required non-owning observer. |
| Object is required and read-only | `const T&` | Required non-owning observer. |
| Object is optional and may be modified | `T*` | Nullable non-owning observer. |
| Object is optional and read-only | `const T*` | Nullable non-owning observer. |

Later lessons introduce `std::unique_ptr` and `std::shared_ptr` for APIs that
transfer or share ownership. Do not use a smart pointer merely to say
"optional observer."

## Observers do not extend lifetime

```cpp
Sensor* observer = nullptr;

{
    Sensor temporary{"temporary", 10.0};
    observer = &temporary;
} // temporary is destroyed here

// observer is now dangling; do not dereference it.
```

The pointer variable still exists, but its address no longer identifies a live
`Sensor`. Checking `observer != nullptr` is not enough: a dangling pointer is
usually non-null.

The owner must outlive every observer.

## Predict the behavior

Before opening the answer, predict the complete output and identify the owner:

```cpp
#include <iostream>
#include <string>
#include <utility>

class Device
{
public:
    explicit Device(std::string name) : name_(std::move(name))
    {
        std::cout << "Create " << name_ << '\n';
    }

    ~Device()
    {
        std::cout << "Destroy " << name_ << '\n';
    }

    void ping() const
    {
        std::cout << "Ping " << name_ << '\n';
    }

private:
    std::string name_;
};

void inspect(const Device& device)
{
    device.ping();
}

void inspect_optional(const Device* device)
{
    if (device != nullptr)
        device->ping();
    else
        std::cout << "No device\n";
}

int main()
{
    Device camera{"camera"};
    inspect(camera);
    inspect_optional(&camera);
    inspect_optional(nullptr);
}
```

<details>
<summary>Show the expected output</summary>

```text
Create camera
Ping camera
Ping camera
No device
Destroy camera
```

`camera` is the owner. Both function parameters are temporary observers. No
function call copies or destroys the `Device`.

</details>

## Compile the experiment

Save the program as `observer.cpp`, then compile and run it:

```bash
g++ -std=c++20 -Wall -Wextra -Wpedantic observer.cpp -o observer
./observer
```

Try changing `inspect(const Device&)` to `inspect(Device)`. Run it again and
observe that passing by value creates a separate object.

## Small coding exercise

Create a `Motor` structure containing a name and speed. Then write:

1. `print_motor(const Motor&)` for required read-only access.
2. `set_speed(Motor&, int)` for required writable access.
3. `stop_if_present(Motor*)` for optional writable access.

Call `stop_if_present` once with a motor's address and once with `nullptr`.
For every parameter, write a comment stating whether it owns or observes the
`Motor`.

## Quiz

### 1. Who owns the object?

```cpp
Sensor sensor{"imu", 4.0};
Sensor& view = sensor;
```

<form class="observer-quiz" data-answer="sensor" data-explanation="The local variable sensor controls the object's lifetime. view is only another name for that object.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="obq1" value="sensor"><code>sensor</code></label>
    <label><input type="radio" name="obq1" value="view"><code>view</code></label>
    <label><input type="radio" name="obq1" value="both">Both own separate objects</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="observer-quiz-result" aria-live="polite"></p>
</form>

### 2. Which parameter means required, read-only observation?

<form class="observer-quiz" data-answer="const-ref" data-explanation="const T&amp; requires an object, avoids copying it, and prevents modification through the parameter.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="obq2" value="value"><code>Sensor</code></label>
    <label><input type="radio" name="obq2" value="const-ref"><code>const Sensor&amp;</code></label>
    <label><input type="radio" name="obq2" value="pointer"><code>Sensor*</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="observer-quiz-result" aria-live="polite"></p>
</form>

### 3. Which parameter expresses optional, read-only observation?

<form class="observer-quiz" data-answer="const-pointer" data-explanation="const T* can be nullptr and prevents modification of the observed object through that pointer.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="obq3" value="const-pointer"><code>const Sensor*</code></label>
    <label><input type="radio" name="obq3" value="const-ref"><code>const Sensor&amp;</code></label>
    <label><input type="radio" name="obq3" value="value"><code>const Sensor</code></label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="observer-quiz-result" aria-live="polite"></p>
</form>

### 4. Is every non-null pointer safe to dereference?

<form class="observer-quiz" data-answer="no" data-explanation="A dangling pointer can remain non-null after the pointed-to object's lifetime ends. The owner must still be alive.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="obq4" value="yes">Yes, non-null always means valid</label>
    <label><input type="radio" name="obq4" value="no">No, it may point to an object that has been destroyed</label>
    <label><input type="radio" name="obq4" value="const">Only const pointers are safe</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="observer-quiz-result" aria-live="polite"></p>
</form>

### 5. Should an observer call `delete` on the observed object?

<form class="observer-quiz" data-answer="never" data-explanation="An observer is not responsible for destruction. Deleting an object it does not own can cause invalid deletion or double deletion.">
  <fieldset>
    <legend>Choose one answer:</legend>
    <label><input type="radio" name="obq5" value="always">Yes, after every use</label>
    <label><input type="radio" name="obq5" value="nullable">Only when the pointer is non-null</label>
    <label><input type="radio" name="obq5" value="never">No, cleanup belongs to the owner</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="observer-quiz-result" aria-live="polite"></p>
</form>

<style>
.observer-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.observer-quiz-result { padding: .6rem; }
.observer-quiz-result:empty { display: none; }
.observer-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.observer-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.observer-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.observer-quiz-result');

    if (!selected) {
      result.className = 'observer-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `observer-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Code-review challenge

Review this function:

```cpp
void show_sensor(Sensor* sensor)
{
    if (sensor == nullptr)
        return;

    std::cout << sensor->name << '\n';
    delete sensor;
}

int main()
{
    Sensor temperature{"temperature", 23.5};
    show_sensor(&temperature);
}
```

What ownership mistake does it make?

<details>
<summary>Show the review</summary>

`show_sensor` receives a non-owning observer but calls `delete` on it. The
pointed-to `Sensor` is a local object and was not created with `new`; deleting
it causes undefined behavior. It would also conflict with the real owner's
cleanup responsibility.

The observer should only use the object:

```cpp
void show_sensor(const Sensor& sensor)
{
    std::cout << sensor.name << '\n';
}
```

A reference is clearer here because the object is required. The local
`temperature` object remains responsible for its own lifetime.

</details>

## Lesson summary

- Owners control resource lifetime and cleanup.
- References are useful for required, non-owning access.
- Pointers are useful for optional, non-owning access.
- `nullptr` means that a pointer observes no object.
- A non-null pointer may still dangle if the owner has been destroyed.
- Observers must not destroy the objects they observe.

Next: RAII and automatic resource cleanup.

