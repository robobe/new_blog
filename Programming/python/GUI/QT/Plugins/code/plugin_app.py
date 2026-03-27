from __future__ import annotations

import importlib
import pkgutil
import sys
from typing import List, cast

from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QTabWidget,
    QWidget,
)

from plugin_api import PluginHandle


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


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Qt MVP Plugin Tabs")
        self.resize(640, 360)

        # Tab widget to hold plugins
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Load plugins
        self._plugins = load_plugins(self.tabs)
        if not self._plugins:
            self.tabs.addTab(QLabel("No plugins found in ./plugins"), "Empty")
        else:
            for plugin in self._plugins:
                self.tabs.addTab(plugin.widget, plugin.name)

    def closeEvent(self, a0) -> None:
        for plugin in self._plugins:
            if plugin.dispose:
                plugin.dispose()
        super().closeEvent(a0)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
