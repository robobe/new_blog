---
tags:
    - python
    - vscode
    - dev_env
---

# VSCode python dev env.
Using vscode and `uv` to setup a python development environment.

## create project using uv

```bash
uv init
# create venv install pytest and update pyproject.toml

```

## pytest

```bash
uv add --dev pytest
```

### pyproject.toml
Add pytest settings

```toml
[tool.pytest.ini_options]
pythonpath = [
  "."
]
```

## VSCode

### Extensions
- ruff


### Settings

#### VSCode python autocomplete

```json
"python.analysis.extraPaths": [
    "${workspaceFolder}/src"
]
```

#### ruff
```json
"ruff.importStrategy": "useBundled",
"editor.defaultFormatter": "charliermarsh.ruff",
"editor.formatOnPaste": true,
"editor.formatOnSave": true,
"editor.formatOnSaveMode": "file",
"editor.codeActionsOnSave": {
    "source.organizeImports": "always",
    "source.fixAll": "always"
},
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