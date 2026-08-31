---
title: Object Lifetime and Scope
tags:
    - cpp
    - ownership
    - lifetime
    - scope
---

Before choosing a smart pointer, you must know when a C++ object begins to
exist and when it is destroyed.

## Learning goal

After this lesson, you should be able to explain:

- the difference between an object's **lifetime** and a name's **scope**;
- when local objects are constructed and destroyed;
- why leaving a block performs automatic cleanup;
- why a pointer does not keep the object it points to alive.

## Start with a simple example

```cpp
#include <iostream>

class Lamp
{
public:
    Lamp()
    {
        std::cout << "Lamp created\n";
    }

    ~Lamp()
    {
        std::cout << "Lamp destroyed\n";
    }
};

int main()
{
    Lamp desk_lamp;
    std::cout << "Using the lamp\n";
}
```

Output:

```text
Lamp created
Using the lamp
Lamp destroyed
```

`desk_lamp` is created when execution reaches its declaration. It is destroyed
automatically when `main` ends. No manual cleanup is required.

## Scope and lifetime are different

**Scope** describes where a name can be used in the source code.

**Lifetime** describes the period during execution when an object exists and
may be safely used.

```cpp
int main()
{
    int outside = 10;

    {
        int inside = 20;
        std::cout << outside + inside << '\n';
    }

    // inside is out of scope here, and its object has been destroyed.
    std::cout << outside << '\n';
}
```

For these local variables, scope and lifetime end at the same closing brace.
That is common, but the terms do not mean the same thing. Later lessons will
show pointers whose names remain in scope after the objects they point to have
already been destroyed.

## Nested scopes

A pair of braces creates a block. An object declared inside a block is cleaned
up when execution leaves that block, including when the function returns early
or throws an exception.

```cpp
{
    Lamp first;

    {
        Lamp second;
    } // second is destroyed here

} // first is destroyed here
```

Objects in the same scope are destroyed in the reverse order of their completed
construction. This matters when one object depends on another.

## Predict the behavior

Before opening the answer, write down the exact output:

```cpp
#include <iostream>
#include <string>
#include <utility>

class Tracer
{
public:
    explicit Tracer(std::string name) : name_(std::move(name))
    {
        std::cout << "Create " << name_ << '\n';
    }

    ~Tracer()
    {
        std::cout << "Destroy " << name_ << '\n';
    }

private:
    std::string name_;
};

int main()
{
    Tracer first{"first"};

    {
        Tracer second{"second"};
        Tracer third{"third"};
    }

    std::cout << "End of main\n";
}
```

<details>
<summary>Show the expected output</summary>

```text
Create first
Create second
Create third
Destroy third
Destroy second
End of main
Destroy first
```

`third` is destroyed before `second` because they were constructed in the
opposite order. `first` lives until the end of `main`.

</details>

## Compile the experiment

Save the program as `lifetime.cpp`, then compile and run it:

```bash
g++ -std=c++20 -Wall -Wextra -Wpedantic lifetime.cpp -o lifetime
./lifetime
```

Try moving `third` outside the inner braces. Predict its new destruction point
before compiling again.

## Ownership begins with cleanup responsibility

The owner of a resource is responsible for releasing it. A local value normally
owns its own resources and cleans them up in its destructor:

```cpp
void display_message()
{
    std::string message{"Hello"};
    std::cout << message << '\n';
} // message releases its memory automatically
```

This is the foundation of **RAII** and smart pointers. Smart pointers are useful
later when an object must be owned indirectly, but a normal value is the
simplest choice when it works.

## A pointer does not extend lifetime

```cpp
int* observer = nullptr;

{
    int value = 42;
    observer = &value; // observer does not own value
} // value is destroyed

// Do not dereference observer here. It is a dangling pointer.
```

The variable `observer` is still in scope, but the `value` object no longer
exists. A pointer only stores an address; it does not automatically own the
object or keep it alive.

## Small coding exercise

Create a `Book` class that:

1. Stores a title.
2. Prints `Open: TITLE` in its constructor.
3. Prints `Close: TITLE` in its destructor.
4. Creates one `Book` in `main` and two more inside a nested block.

Before running it, predict the destruction order. Then add an early `return`
inside the nested block and confirm that all constructed books are still
destroyed.

## Quiz

### 1. What does object lifetime describe?

<form class="lifetime-quiz" data-answer="exists" data-explanation="Lifetime is the period during execution when an object exists and may be safely used.">
  <fieldset>
    <legend>Select one answer.</legend>
    <label><input type="radio" name="q1" value="visible"> Where a variable name is visible in source code</label>
    <label><input type="radio" name="q1" value="exists"> When an object exists during program execution</label>
    <label><input type="radio" name="q1" value="heap"> Whether an object uses heap memory</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="lifetime-quiz-result" aria-live="polite"></p>
</form>

### 2. When is a normal local object destroyed?

<form class="lifetime-quiz" data-answer="block" data-explanation="A local object with automatic storage duration is destroyed when execution leaves its block.">
  <fieldset>
    <legend>Select one answer.</legend>
    <label><input type="radio" name="q2" value="program"> Only when the program terminates</label>
    <label><input type="radio" name="q2" value="manual"> Only after calling delete</label>
    <label><input type="radio" name="q2" value="block"> When execution leaves its block</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="lifetime-quiz-result" aria-live="polite"></p>
</form>

### 3. In what order are local objects in one scope destroyed?

<form class="lifetime-quiz" data-answer="reverse" data-explanation="Local objects are destroyed in reverse order of their completed construction.">
  <fieldset>
    <legend>Select one answer.</legend>
    <label><input type="radio" name="q3" value="same"> In construction order</label>
    <label><input type="radio" name="q3" value="reverse"> In reverse construction order</label>
    <label><input type="radio" name="q3" value="random"> In an unspecified random order</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="lifetime-quiz-result" aria-live="polite"></p>
</form>

### 4. Does a raw pointer keep a local object alive?

<form class="lifetime-quiz" data-answer="no" data-explanation="A raw pointer only stores an address. It does not extend the pointed-to object's lifetime.">
  <fieldset>
    <legend>Select one answer.</legend>
    <label><input type="radio" name="q4" value="yes"> Yes, until the pointer is reset</label>
    <label><input type="radio" name="q4" value="no"> No, the object still dies at the end of its lifetime</label>
    <label><input type="radio" name="q4" value="const"> Only if the pointer is const</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="lifetime-quiz-result" aria-live="polite"></p>
</form>

### 5. What is the simplest ownership choice for a local object?

<form class="lifetime-quiz" data-answer="value" data-explanation="Prefer a normal value when possible. Its destructor provides automatic and deterministic cleanup.">
  <fieldset>
    <legend>Select one answer.</legend>
    <label><input type="radio" name="q5" value="raw"> Allocate it with new and store a raw pointer</label>
    <label><input type="radio" name="q5" value="shared"> Always use shared_ptr</label>
    <label><input type="radio" name="q5" value="value"> Store it as a normal value</label>
  </fieldset>
  <button type="submit">Check answer</button>
  <p class="lifetime-quiz-result" aria-live="polite"></p>
</form>

<style>
.lifetime-quiz fieldset { display: grid; gap: .4rem; margin-bottom: .7rem; }
.lifetime-quiz-result { padding: .6rem; }
.lifetime-quiz-result:empty { display: none; }
.lifetime-quiz-result.correct { background: #dff5e3; color: #176b2c; }
.lifetime-quiz-result.incorrect { background: #fde2e2; color: #9b1c1c; }
</style>

<script>
document.querySelectorAll('.lifetime-quiz').forEach((quiz) => {
  quiz.addEventListener('submit', (event) => {
    event.preventDefault();
    const selected = quiz.querySelector('input:checked');
    const result = quiz.querySelector('.lifetime-quiz-result');

    if (!selected) {
      result.className = 'lifetime-quiz-result incorrect';
      result.textContent = 'Choose an answer first.';
      return;
    }

    const correct = selected.value === quiz.dataset.answer;
    result.className = `lifetime-quiz-result ${correct ? 'correct' : 'incorrect'}`;
    result.textContent = `${correct ? 'Correct.' : 'Not quite.'} ${quiz.dataset.explanation}`;
  });
});
</script>

## Code-review challenge

What is wrong with this function?

```cpp
#include <string>

const std::string* make_name()
{
    std::string name{"Ada"};
    return &name;
}
```

<details>
<summary>Show the review</summary>

`name` is destroyed when `make_name` returns. The returned pointer therefore
points to an object that no longer exists. Dereferencing it causes undefined
behavior.

Return the string by value:

```cpp
std::string make_name()
{
    return "Ada";
}
```

The caller receives its own valid `std::string`. No pointer or smart pointer is
needed.

</details>

## Lesson summary

- Scope controls where a name can be used.
- Lifetime controls when an object exists.
- Local objects are destroyed automatically when execution leaves their block.
- Local objects in one scope are destroyed in reverse construction order.
- A pointer does not keep another object alive.
- Prefer normal values when they express the required ownership.

Next: raw pointers and references as non-owning observers.

