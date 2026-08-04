---
title: Grill Me Skills in Codex
tags:
    - codex
    - skills
    - planning
    - ai-agent
---

# Grill Me Skills in Codex

`grill-me` and `grill-with-docs` are community skills from
[mattpocock/skills](https://github.com/mattpocock/skills). They make Codex
interview you before implementation so vague ideas, hidden assumptions, and
important decisions become clear.

!!! note
    The documentation-aware skill is named **`grill-with-docs`**, although it
    is sometimes informally called “grill-me-doc.”

## What each skill does

### `grill-me`

`grill-me` runs a focused interview about a plan, design, or idea. Codex asks
one question at a time, recommends an answer, and follows each decision until
both sides share the same understanding.

Use it when:

- the idea is still unclear;
- you want assumptions challenged before coding;
- you do not need the answers written into project documentation.

### `grill-with-docs`

`grill-with-docs` runs the same interview and also uses domain modeling to
record useful results in the repository:

- resolved project terminology goes into `CONTEXT.md`;
- important, hard-to-reverse decisions can become Architecture Decision
  Records (ADRs) under `docs/adr/`.

Use it for a real software project when the discussion should improve both the
plan and the project's long-term documentation. These files are created only
when useful, but this skill can modify the repository while the interview is
running.

## How is this different from Plan mode?

| | Grill skills | Codex Plan mode |
| --- | --- | --- |
| Type | Reusable community workflow | Built-in Codex mode |
| Main purpose | Stress-test your thinking through a deliberate interview | Gather context and produce an implementation plan |
| Question style | One decision at a time, with assumptions actively challenged | Clarifying questions as needed to complete the plan |
| Repository output | `grill-with-docs` may update `CONTEXT.md` and ADRs | Focuses on planning instead of implementing the change |
| Best use | The desired behavior or design is still uncertain | The goal is understood and you need a safe execution plan |

They are complementary. Use a grill skill to decide **what should be built and
why**, then use Plan mode to define **how it should be implemented**.

## Install for Codex

The wrapper skills depend on `grilling`; `grill-with-docs` also depends on
`domain-modeling`. Install all four so both workflows are complete.

Install globally to make them available in every project:

```bash
npx skills@latest add mattpocock/skills \
    --skill grill-me \
    --skill grill-with-docs \
    --skill grilling \
    --skill domain-modeling \
    --agent codex \
    --global
```

For a team-shared, repository-specific installation, run the command from the
repository root and omit `--global`:

```bash
npx skills@latest add mattpocock/skills \
    --skill grill-me \
    --skill grill-with-docs \
    --skill grilling \
    --skill domain-modeling \
    --agent codex
```

Review third-party skill instructions before installing them. Codex normally
detects new skills automatically; restart Codex if they do not appear. Run
`/skills` in the Codex CLI to confirm that they are available.

## Usage in Codex

Codex CLI and the IDE extension use `$` to explicitly select a skill.

Use `grill-me` for an idea without documentation changes:

```text
$grill-me I want to add offline synchronization to my application.
Interview me until the behavior and trade-offs are clear. Do not implement it.
```

Use `grill-with-docs` inside a repository:

```text
$grill-with-docs Stress-test my plan to split billing from the main service.
Use the existing code and documentation as context. Do not implement the change.
```

Answer each question before Codex asks the next one. When the interview reaches
shared understanding, review any generated `CONTEXT.md` or ADR changes with
`git diff`.

### Combine it with Plan mode

For a larger change:

1. Enter Plan mode with `/plan`.
2. Invoke `$grill-me` or `$grill-with-docs` with the idea.
3. Complete the interview one question at a time.
4. Ask Codex to turn the agreed decisions into an implementation plan.
5. Review the plan, leave Plan mode, and request implementation only when ready.

This combination gives you the grill skill's deeper decision interview and
Plan mode's built-in planning boundary.

## References

- [Matt Pocock's skills repository](https://github.com/mattpocock/skills)
- [Codex skill documentation](https://developers.openai.com/codex/skills)
- [Codex CLI slash commands](https://developers.openai.com/codex/cli/slash-commands)
- [`npx skills` installer](https://github.com/vercel-labs/skills)
