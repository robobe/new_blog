# C++ Mastery Course Instructions

These instructions apply to files under this directory. Navigation pages,
planning pages, and reference summaries do not need to follow the full lesson
template.

## Course purpose

Create posts that help a programmer learn modern C++ by understanding behavior,
making design choices, compiling experiments, and reviewing code. Do not teach
modern C++ as a list of new syntax.

Each lesson should answer:

1. What problem does this feature solve?
2. What did programmers commonly do before it?
3. What ownership, lifetime, type, or error guarantees does it provide?
4. When should it be used?
5. When is a simpler or older tool clearer?
6. What mistake is a beginner likely to make?

The target reader knows basic C++ syntax and wants to progress toward practical
C++20. Introduce terminology in plain language and then use its correct C++
name.

## Sources of truth

- Follow the dependency order and lesson contract in `plan.md`.
- Use `index.md` to track available phases and progress.
- Use `07_modern_cpp/index.md` as the topic menu for standalone modern C++
  feature posts.
- Match the interactive quiz behavior in `07_modern_cpp/optional.md`: correct
  feedback is green, incorrect feedback is red, and every answer includes an
  explanation.
- Link existing material instead of rewriting the same explanation in another
  lesson.

Do not mark a lesson or phase as available until its examples, exercise, tests,
quiz, links, and documentation build have been checked.

## Learning order

Keep prerequisite topics before the features that depend on them:

1. Toolchain, warnings, debugging, and multi-file programs
2. Values, expressions, control flow, functions, text, and data modeling
3. Iterators, algorithms, containers, and vocabulary types
4. Errors, tests, undefined behavior, sanitizers, and program design
5. Compile-time and generic programming
6. Concurrency and cancellation
7. Packaging, compatibility, benchmarking, and profiling
8. Coroutines and modules as specializations

Within a topic, start with the common use case. Add advanced details only after
the basic behavior and tradeoff are clear.

## Lesson structure

Build each lesson around one main idea and use this learning cycle:

1. **Goal and prerequisites** — state what the reader will learn and what they
   should already know.
2. **Brief example** — show the smallest useful example before detailed rules.
3. **Explain the model** — describe what the compiler or program does and why.
4. **Compare choices** — show when to use the feature and when not to use it.
5. **Predict behavior** — ask the reader to predict output, types, lifetime, or
   compilation results before revealing the answer.
6. **Compile experiments** — include 2–4 small, complete programs and expected
   output where useful.
7. **Practice** — provide a compiling starter exercise and a focused task.
8. **Check understanding** — finish with 5–10 multiple-choice questions.
9. **Review code** — ask the reader to find a realistic bug or design problem.
10. **Completion check** — list what the reader should now be able to explain or
    implement.

Put answers, reasoning, and exercise solutions in collapsible `<details>`
sections so readers attempt the problem first. A phase mission may provide hints
and acceptance tests, but must not provide a complete solution.

Aim for 15–25 minutes of focused instruction. Split a post when it develops a
second independent learning goal.

## Modern C++ teaching principles

- Prefer values and the Rule of Zero before pointers or manual resource
  management.
- Make ownership, lifetime, mutation, and nullability visible in examples.
- Prefer RAII and standard-library facilities over handwritten cleanup or
  utilities.
- Prefer algorithms and ranges when they clarify intent; keep a loop when it is
  easier to understand.
- Explain copies, moves, references, views, and invalidation whenever a feature
  can hide them.
- Treat `auto` as type deduction, not dynamic typing, and show when an explicit
  type communicates intent better.
- Treat `string_view`, `span`, iterators, ranges, and views as non-owning. State
  the lifetime requirement near the example.
- Use `optional` for normal absence, `expected` for an expected failure with
  error information, and exceptions for failures that cannot be handled
  locally. Explain the choice instead of presenting it as a universal rule.
- Use concepts to express real template requirements, not to decorate
  unconstrained examples.
- Teach concurrency with shutdown, cancellation, shared-state, and failure
  behavior. Never present a detached thread as the easy default.
- Show unsafe or obsolete techniques only to explain a failure or migration.
  Clearly label them and follow them immediately with the preferred approach.
- Avoid macros, custom framework code, and unrelated template machinery in
  beginner examples.

## Language-version rules

- Use C++20 by default.
- State the first standard that introduced the feature, such as C++11, C++17,
  or C++20.
- Mark `std::expected` examples as C++23 and compile them separately.
- Keep modules conceptual until the supported toolchain workflow is documented.
- Do not use a newer language or library feature silently in an older-standard
  example.

## Code and verification

- Keep examples complete, warning-free, and small enough to type by hand.
- Prefer the standard library before adding a dependency or custom helper.
- Use descriptive domain names such as `reading`, `sensor`, or `message` rather
  than unexplained `foo` and `bar`.
- Show expected output when it teaches behavior. Do not promise an exact order
  when the C++ standard does not guarantee one.
- Compile ordinary examples with:

  ```bash
  g++ -std=c++20 -Wall -Wextra -Wpedantic example.cpp -o example
  ```

- Give each full runnable lesson a local `code/CMakeLists.txt`, a compiling
  `exercise_starter.cpp`, and the smallest useful CTest checks, as required by
  `plan.md`.
- Test deterministic output and at least one relevant invalid or failure path.
- Use AddressSanitizer and UndefinedBehaviorSanitizer in reliability lessons;
  use ThreadSanitizer in concurrency lessons when supported.
- Build the documentation after adding a lesson and fix new page-specific
  warnings.

Do not copy large code blocks into Markdown and `code/` independently. Keep one
canonical runnable source when practical, and ensure any shortened Markdown
version remains behaviorally equivalent.

## Questions and quizzes

Include a useful mix of:

- predict-the-output;
- predict-the-deduced-type or compile result;
- choose-the-right-tool;
- find-the-lifetime or ownership bug;
- find undefined behavior;
- make a small code change.

Wrong-answer feedback must teach the correct rule, not merely say that the
choice is wrong. Use distinct form names and CSS class prefixes per page so
multiple quizzes do not interfere with one another.

## Navigation and maintenance

- Add every lesson to its nearest `index.md` menu.
- Add prerequisites and links to the next logical lesson.
- Use the existing numbered phase directories for curriculum lessons.
- Use lowercase descriptive filenames for posts inside a phase.
- Update status tables only after verification is complete.
- Preserve valid existing material and fix it only when it conflicts with the
  lesson's learning goal or contains an error.

## Lesson completion check

A lesson is complete when the reader can explain the feature's purpose, predict
its important behavior, choose it over relevant alternatives, compile a small
example, complete the exercise, and identify its most common misuse. The page
must also satisfy the tests and documentation checks defined in `plan.md`.
