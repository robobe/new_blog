---
title: Python runtime plugin loading
tags:
    - python
    - plugin
    - importlib
    - protocol
    - typing
---

Python normally loads modules through `import` statements, but it can also load a
module from a file path at runtime. This is useful for plugin systems where the
main application does not know every plugin module when the application starts.

The demo in this page uses four files:

- `plugin_api.py` defines the plugin interface.
- `my_plugin.py` implements a plugin.
- `plugin_loader.py` loads and validates a plugin module from a path.
- `main.py` calls the loader and uses the plugin.

## Protocol

`Plugin` is a `Protocol`. A protocol describes the shape an object must have
without forcing the object to inherit from a base class.

In this demo, a valid plugin instance must have:

- a `name` attribute
- a `setup(config)` method
- a `process(data)` method

The protocol is decorated with `@runtime_checkable`, which makes this possible:

```python
isinstance(plugin, Plugin)
```

Without `@runtime_checkable`, protocols are mainly for static type checkers and
cannot be used with `isinstance()`.

<details>
<summary>plugin_api.py</summary>

```python
--8<-- "docs/Programming/python/python/plugin/code/plugin_api.py"
```

</details>

## Plugin module

The plugin file defines a normal Python class. It does not inherit from
`Plugin`; it only needs to match the same structure.

The `register()` function is the plugin contract. The loader expects every
plugin module to expose a callable `register()` function that returns the plugin
class.

<details>
<summary>my_plugin.py</summary>

```python
--8<-- "docs/Programming/python/python/plugin/code/my_plugin.py"
```

</details>

## Runtime load flow

The loader receives a file path and imports that file as a Python module:

1. `Path(path).resolve()` converts the plugin path to an absolute path.
2. The loader builds a stable unique module name from the file stem and a hash
   of the resolved path.
3. `importlib.util.spec_from_file_location()` creates an import specification
   for that file and module name.
4. `importlib.util.module_from_spec()` creates an empty module object.
5. The module is registered in `sys.modules` before execution, matching normal
   import behavior.
6. `spec.loader.exec_module(module)` executes the file and fills the module
   object with its classes, functions, and variables. If execution fails, the
   loader removes the failed module from `sys.modules` or restores the previous
   entry.
7. `getattr(module, "register", None)` finds the plugin registration function.
8. `inspect.signature(register)` validates that `register()` does not require
   arguments.
9. `register()` returns the plugin class.
10. The loader creates an instance and validates it with `isinstance(plugin, Plugin)`.
11. `setup(config)` initializes the plugin before returning it to the caller.

<details>
<summary>plugin_loader.py</summary>

```python
--8<-- "docs/Programming/python/python/plugin/code/plugin_loader.py"
```

</details>

## Runtime validation

The important check is:

```python
if not isinstance(plugin, Plugin):
    raise TypeError("Plugin does not match Plugin protocol")
```

Because `Plugin` uses `@runtime_checkable`, `isinstance()` performs a structural
runtime check. The plugin object does not need to inherit from `Plugin`; it only
needs the required attributes to exist.

This catches simple invalid plugins before the application calls `setup()` or
`process()`. For example, a plugin without `process()` fails the check.

!!! warning
    `@runtime_checkable` only checks that required attributes exist at runtime.
    It does not validate method signatures, parameter types, return types, or
    attribute types. Static type checking with tools like pyright or mypy is
    still useful.

## Usage

The application gives the loader a path to the plugin file and a configuration
dictionary. The returned object is typed as `Plugin`, so the application can call
the agreed API without knowing the concrete plugin class.

<details>
<summary>main.py</summary>

```python
--8<-- "docs/Programming/python/python/plugin/code/main.py"
```

</details>

Running the demo prints:

```text
50
```

---

## Reusable skill guide

This example also includes a reusable Codex skill:
[SKILL.md](code/SKILL.md).

The skill turns the demo into an implementation checklist that can be copied to
another project. Its main ideas are:

- Keep the plugin API small and explicit with a `Protocol`.
- Use `@runtime_checkable` only when the loader needs runtime validation with
  `isinstance()`.
- Make each plugin module expose a no-argument `register()` function.
- Make `register()` return the plugin class, so the loader controls
  construction, setup, and validation.
- Load plugin files with `importlib.util.spec_from_file_location()`.
- Register runtime-loaded modules in `sys.modules` before execution, and roll
  back that entry if execution fails.
- Generate module names from the resolved path, not only the filename, so two
  plugins named `my_plugin.py` in different folders do not collide.
- Validate invalid plugin shapes early and raise clear errors.

<details>
<summary>SKILL.md</summary>

```markdown
--8<-- "docs/Programming/python/python/plugin/code/SKILL.md"
```

</details>

