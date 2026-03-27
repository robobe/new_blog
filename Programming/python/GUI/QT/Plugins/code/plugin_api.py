from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional

from PyQt6.QtWidgets import QWidget


@dataclass
class PluginHandle:
    plugin_id: str
    name: str
    widget: QWidget
    presenter: object
    dispose: Optional[Callable[[], None]] = None
