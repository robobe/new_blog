---
title: VSCode python dev environment using uv and ruff
tags:
    - python
    - vscode
    - dev_env
    - uv
    - ruff
    - pre-commit
---

uv is a fast Python package manager

## install

```bash
curl -Ls https://astral.sh/uv/install.sh | sh
```


## basic usage

### Create project with venv automatically

```
uv init
```

### Add dependency

```bash
uv add numpy
```

#### add dev dependency

```bash
uv add --dev pytest
```


### Run script inside project env

```bash
uv run python app.py
```

!!! tip "no need to activate venv manually"
    

### Install from lock file
Reproducible install.

```bash
uv sync
```

---

## Lock file

A lock file freezes the **exact** versions of all dependencies — including transitive dependencies.

It guarantees:

**“Everyone installs exactly the same environment.”**




| File             | Purpose               |
| ---------------- | --------------------- |
| `pyproject.toml` | What you WANT         |
| `uv.lock`        | What you ACTUALLY GET |


### install exactly locked versions

```bash
uv sync
```

!!! info "Transitive Dependencies"
    for example install `uv add fastapi`
    it install many depend packages

    Lock file records ALL of them
    

---

## VSCode

### Extensions
- [ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff): Python linter and formatter .


### Settings

#### VSCode python autocomplete

```json
"python.analysis.extraPaths": [
    "${workspaceFolder}/src"
]
```

#### ruff

```json title="disabled python ext linting"
"python.linting.enabled": false,
```

```json
"ruff.importStrategy": "fromEnvironment",
"editor.defaultFormatter": "charliermarsh.ruff",
"editor.formatOnPaste": true,
"editor.formatOnSave": true,
"editor.formatOnSaveMode": "file",
"editor.codeActionsOnSave": {
    "source.organizeImports": "always",
    "source.fixAll": "always"
},
"python.linting.enabled": false,
```

#### type checking

```json
"python.analysis.typeCheckingMode": "strict"
```

---

## ruff settings

```ini title="ruff.toml"
line-length = 88
indent-width = 4

[format]
quote-style = "double"
indent-style = "space"
skip-magic-trailing-comma = false
line-ending = "auto"
docstring-code-format = true
docstring-code-line-length = "dynamic"
```

### vscode settings.json

```json
{
    // -----------------------------
    // 🧹 Formatting (Ruff)
    // -----------------------------
    "editor.defaultFormatter": "charliermarsh.ruff",
    "editor.formatOnSave": true,
    "editor.formatOnPaste": true,
    "editor.formatOnSaveMode": "file",

    "editor.codeActionsOnSave": {
        "source.organizeImports": "always",
        "source.fixAll": "always"
    },

    "ruff.importStrategy": "fromEnvironment",
    "ruff.lint.enable": true,
    "ruff.format.enable": true,

    // -----------------------------
    // 🧠 Type Checking (Pyright)
    // -----------------------------
    "python.analysis.typeCheckingMode": "strict",
    "python.analysis.autoImportCompletions": true,
    "python.analysis.inlayHints.variableTypes": true,
    "python.analysis.inlayHints.functionReturnTypes": true,

    // -----------------------------
    // 🐍 Python Behavior
    // -----------------------------
    "python.linting.enabled": false,
    "python.testing.pytestEnabled": true,
    "python.testing.unittestEnabled": false,

    // -----------------------------
    // 🔍 Editor Quality of Life
    // -----------------------------
    "files.trimTrailingWhitespace": true,
    "files.insertFinalNewline": true,
    "editor.rulers": [100],
    "editor.minimap.enabled": false,
    "editor.renderWhitespace": "boundary",

    // -----------------------------
    // 🧪 Optional: pytest auto-discovery
    // -----------------------------
    "python.testing.pytestArgs": [
        "tests"
    ]
}
```

---

## pre-commit

pre-commit runs checks automatically before git commit.

If checks fail → commit is blocked.

It prevents:

- Broken formatting
- Unused imports
- Simple lint errors
- Bad whitespace
- Committing junk


### install

```bash
uv add --dev pre-commit
```

### config

```yaml
repos:
  - repo: https://github.com/astral-sh/ruff-pre-commit
    rev: v0.4.7   # use latest
    hooks:
      - id: ruff
        args: ["--fix"]
      - id: ruff-format

  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.6.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-yaml
      - id: check-toml

```

### Activate and usage

```bash
uv run pre-commit install
```

Now it run the pre-commit script every `git commit`

#### Run manually

```bash
uv run pre-commit run --all-files
```