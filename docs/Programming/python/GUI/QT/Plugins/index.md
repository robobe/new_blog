---
title: QT Plugins - Load widget dynamic
tags:   
    - qt
    - plugin
---

<details>
<summary>main.py</summary>
```
--8<-- "docs/Programming/GUI/QT/Plugins/code/plugin_app.py"
```
</details>

### Dynamic discovert
Dynamically discover and load plugin modules from a plugins package, call a factory function (create_plugin) in each plugin, and return a list of instantiated plugin objects.

```python
def load_plugins(parent: QWidget) -> List[PluginHandle]:
    plugins: List[PluginHandle] = []
    pkg = importlib.import_module("plugins")

    for mod in pkgutil.iter_modules(pkg.__path__):
        if mod.ispkg:
            continue
        try:
            module = importlib.import_module(f"plugins.{mod.name}")
            create_plugin = getattr(module, "create_plugin", None)
            if not callable(create_plugin):
                continue
            plugin = create_plugin(parent)
            plugin = cast(PluginHandle, plugin)
            plugins.append(plugin)
        except Exception as exc:
            print(f"[plugin_app] Failed to load plugins.{mod.name}: {exc}")

    return plugins
```

#### import plugins package
- Dynamically imports the package, not individual plugins
- Equivalent to: `import packages`

```python
pkg = importlib.import_module("plugins")
```

#### import each plugin module
- executes the plugin module
- registers its symbols in memory

```python
module = importlib.import_module(f"plugins.{mod.name}")
```

#### factory method

```python
# get method
create_plugin = getattr(module, "create_plugin", None)
# run method
plugin = create_plugin(parent)
```

---

<details>
<summary>plugin_api.py</summary>
```
--8<-- "docs/Programming/GUI/QT/Plugins/code/plugin_api.py"
```
</details>


<details>
<summary>plugins/plug_demo.py</summary>
```
--8<-- "docs/Programming/GUI/QT/Plugins/code/plugins/led_off.py"
```
</details>