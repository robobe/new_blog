---
name: python-runtime-plugin
description: Implement Python plugin systems that load plugin modules from file paths at runtime using importlib, a register() function contract, a typing.Protocol interface, and @runtime_checkable isinstance validation. Use when adding or reviewing dynamic plugin loading in a Python project.
---

# Python Runtime Plugin

Implement plugins with a small explicit contract:

- Define the shared API as a `typing.Protocol`.
- Decorate the protocol with `@runtime_checkable` when runtime validation with `isinstance()` is required.
- Require each plugin module to expose a callable `register()` function.
- Make `register()` return the plugin class, not an instance.
- Load plugin files with `importlib.util.spec_from_file_location()`.
- Instantiate the returned class, validate it against the protocol, then call `setup(config)`.

## Files To Create

Use this structure unless the project already has equivalent module names:

```text
plugin_api.py
plugin_loader.py
plugins/
    my_plugin.py
main.py
```

## Protocol

Create a protocol for the plugin API:

```python
from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class Plugin(Protocol):
    name: str

    def setup(self, config: dict[str, Any]) -> None:
        ...

    def process(self, data: Any) -> Any:
        ...
```

Use `@runtime_checkable` only when runtime checks are needed. It allows:

```python
isinstance(plugin, Plugin)
```

Important limitation: runtime-checkable protocols only check that required
attributes exist. They do not validate method signatures, parameter types,
return types, or attribute types. Keep static type checking enabled when
signature correctness matters.

## Plugin Module Contract

Each plugin module should define a concrete class and a no-argument
`register()` function:

```python
class MyPlugin:
    name = "my_plugin"

    def setup(self, config):
        self.factor = config.get("factor", 1)

    def process(self, data):
        return data * self.factor


def register():
    return MyPlugin
```

The plugin class does not need to inherit from `Plugin`. Matching the protocol
structure is enough.

## Loader Pattern

Use this loader pattern:

```python
import hashlib
import importlib.util
import inspect
import re
import sys
from pathlib import Path
from typing import Any

from plugin_api import Plugin


def _module_name_from_path(file_path: Path) -> str:
    safe_stem = re.sub(r"\W+", "_", file_path.stem).strip("_") or "plugin"
    path_hash = hashlib.sha256(str(file_path).encode()).hexdigest()[:12]
    return f"_runtime_plugin_{safe_stem}_{path_hash}"


def load_plugin_from_file(path: str, config: dict[str, Any]) -> Plugin:
    file_path = Path(path).resolve()
    module_name = _module_name_from_path(file_path)

    spec = importlib.util.spec_from_file_location(
        module_name,
        file_path,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load plugin: {file_path}")

    module = importlib.util.module_from_spec(spec)
    previous_module = sys.modules.get(module_name)
    sys.modules[module_name] = module

    try:
        spec.loader.exec_module(module)
    except Exception:
        if previous_module is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous_module
        raise

    register = getattr(module, "register", None)
    if not callable(register):
        raise TypeError("Plugin module must define a callable register()")

    signature = inspect.signature(register)
    required_parameters = [
        parameter
        for parameter in signature.parameters.values()
        if parameter.default is inspect.Parameter.empty
        and parameter.kind
        in (
            inspect.Parameter.POSITIONAL_ONLY,
            inspect.Parameter.POSITIONAL_OR_KEYWORD,
            inspect.Parameter.KEYWORD_ONLY,
        )
    ]
    if required_parameters:
        raise TypeError("Plugin register() must not require parameters")

    plugin_class = register()
    if not inspect.isclass(plugin_class):
        raise TypeError("Plugin register() must return a plugin class")

    try:
        plugin = plugin_class()
    except TypeError as exc:
        raise TypeError(
            "Plugin class must be constructible without arguments"
        ) from exc

    if not isinstance(plugin, Plugin):
        raise TypeError("Plugin does not match Plugin protocol")

    plugin.setup(config)
    return plugin
```

## Implementation Checklist

When applying this pattern:

- Keep the protocol small and stable.
- Validate the plugin before calling plugin methods.
- Raise clear `ImportError` or `TypeError` messages for invalid plugins.
- Resolve plugin paths before loading them.
- Register runtime-loaded modules in `sys.modules` before execution, and roll
  back that entry if module execution fails.
- Generate module names from the resolved path, not only the filename, so
  plugins with the same basename do not collide.
- Avoid silently importing arbitrary paths from untrusted users.
- Add tests for missing `register()`, invalid `register()` signatures, non-class
  return values, protocol mismatch, and a successful plugin load.

## Usage Example

```python
from pathlib import Path

from plugin_loader import load_plugin_from_file


plugin_path = Path(__file__).resolve().parent / "plugins" / "my_plugin.py"
plugin = load_plugin_from_file(str(plugin_path), {"factor": 10})

print(plugin.process(5))
```
