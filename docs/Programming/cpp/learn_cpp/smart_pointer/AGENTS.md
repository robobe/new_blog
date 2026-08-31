# Smart Pointer Course Instructions

These instructions apply to every file in this directory.

## Course purpose

This course is not primarily about smart-pointer syntax. It teaches C++
ownership and lifetime management, using smart pointers as the main tools.

Every lesson must help the reader answer:

1. What object or resource exists?
2. Who owns it?
3. How long does it live?
4. Who destroys or releases it?
5. Can another part of the program only observe it?

Target readers know basic C++ syntax but are new to ownership. Use C++20 unless
a topic requires another version, and state that requirement beside the code.

## Syllabus

Follow the course order in `index.md`:

1. Object lifetime and scope
2. Raw pointers and references: owner versus observer
3. RAII
4. `std::unique_ptr` basics
5. Moving `std::unique_ptr`
6. `std::unique_ptr` in APIs
7. `std::unique_ptr` in containers
8. Advanced `std::unique_ptr`
9. `std::shared_ptr`
10. `std::weak_ptr`
11. Cyclic ownership
12. Smart pointers in class design
13. Polymorphism and virtual destructors
14. C APIs, external resources, and custom deleters
15. PImpl and incomplete types
16. Ownership-oriented API design
17. Anti-patterns and debugging
18. Final ownership-focused project

Do not introduce `shared_ptr` before exclusive ownership is understood. Teach
`unique_ptr` as the default owning pointer and use `shared_ptr` only when the
design genuinely requires shared lifetime.

## Lesson structure

Build each post around one main ownership idea. Use this learning cycle:

1. **Learn the concept** — begin with a short explanation and a minimal example.
2. **Predict behavior** — ask when objects are destroyed or ownership changes.
3. **Compile an experiment** — provide one complete, runnable program with its
   expected output.
4. **Practice** — give a small coding exercise that changes or completes the
   example.
5. **Check understanding** — finish with 5–10 multiple-choice questions.
6. **Review code** — include a short challenge that asks the reader to find an
   ownership or lifetime problem.

Reveal quiz answers and explanations only after the reader makes a choice. Mark
correct feedback in green and incorrect feedback in red, following the quiz
pattern already used in `../mastery/07_modern_cpp/optional.md`.

## Teaching rules

- Start with automatic storage and deterministic destruction before discussing
  heap allocation.
- Prefer values and RAII types. Introduce a smart pointer only when ownership
  needs to be represented indirectly.
- Prefer `std::make_unique` and `std::make_shared` over direct `new`.
- Avoid owning raw pointers. Label every raw pointer and reference as an owner or
  non-owning observer in the explanation.
- Show ownership transfer explicitly with `std::move` and describe the
  moved-from pointer as valid but empty.
- Explain that `shared_ptr` counts owners, not aliases, observers, or general
  references to the object.
- Introduce `weak_ptr` through a real ownership cycle, not as isolated syntax.
- When teaching polymorphic deletion, require a virtual base destructor.
- For APIs, explain the meaning of each relevant type:
  - `T` stores or transfers a value.
  - `T&` requires a non-null, non-owning object.
  - `T*` is a nullable, non-owning observer unless explicitly documented.
  - `std::unique_ptr<T>` transfers exclusive ownership.
  - `std::shared_ptr<T>` shares lifetime ownership.
  - `std::weak_ptr<T>` observes an object managed by `shared_ptr` without owning it.
- Include failure cases such as leaks, dangling pointers, double deletion,
  use-after-move, and ownership cycles only when they support the current topic.
- Never use `release()` without immediately explaining who becomes responsible
  for cleanup.

## Code and writing standards

- Keep examples small, complete, and compilable.
- Use constructors and destructors that print messages when lifetime order needs
  to be visible.
- Compile examples with warnings enabled:

  ```bash
  g++ -std=c++20 -Wall -Wextra -Wpedantic example.cpp -o example
  ```

- Use sanitizers in debugging lessons:

  ```bash
  g++ -std=c++20 -Wall -Wextra -Wpedantic \
      -fsanitize=address,undefined -g example.cpp -o example
  ```

- State the expected output when it demonstrates construction, destruction, or
  reference-count changes.
- Explain code in plain language before introducing terminology.
- Use diagrams or ownership arrows only when they make relationships clearer.
- Avoid unrelated templates, concurrency, metaprogramming, and custom helper
  abstractions in beginner examples.
- Link every new lesson from `index.md`.

## Lesson completion check

A lesson is complete when it has one clear ownership goal, a runnable example,
an ownership prediction, a small exercise, a multiple-choice quiz with feedback,
and a code-review challenge. The reader should be able to explain who owns each
resource and exactly when it is released.
