# plugin_loader.py
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
    """
    Load a plugin module from a Python source file, instantiate it, and run setup.

    Args:
        path: File system path to the plugin module. The module must define a
            callable register() function that accepts no required parameters.
        config: Configuration passed to the plugin instance through setup().

    Returns:
        A configured object that satisfies the Plugin protocol.

    Raises:
        ImportError: If Python cannot create an import spec or loader for path.
        TypeError: If register() is missing, has required parameters, does not
            return a class, or the created instance does not satisfy Plugin.
    """
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

    # Plugin is a @runtime_checkable Protocol, so isinstance() performs a
    # structural runtime check instead of requiring inheritance. This catches
    # plugins that are missing the API before setup() or process() are called.
    #
    # important
    # ---------
    # @runtime_checkable Protocol checks only that the required attributes exist
    # at runtime. It does not validate method signatures, parameter types,
    # return types, or attribute types.
    if not isinstance(plugin, Plugin):
        raise TypeError("Plugin does not match Plugin protocol")

    plugin.setup(config)
    return plugin
