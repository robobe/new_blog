"""
Minimal PyQt6 MVP example

- Model: plain Python (no Qt)
- View: QWidget + signals (no logic)
- Presenter: connects View ↔ Model

Run:
  pip install PyQt6
  python minimal_mvp.py
"""

from __future__ import annotations
import sys
from dataclasses import dataclass

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QLineEdit,
)


# -------------------------
# Model (Qt-free)
# -------------------------

@dataclass
class GreetingModel:
    name: str = ""

    def greeting(self) -> str:
        n = self.name.strip()
        return f"Hello, {n}!" if n else "Hello!"


# -------------------------
# View (UI only)
# -------------------------

class GreetingView(QWidget):
    # User intent exposed as signals
    nameChanged = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()

        self.edit = QLineEdit()
        self.label = QLabel("Hello!")

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Name:"))
        layout.addWidget(self.edit)
        layout.addWidget(QLabel("Greeting:"))
        layout.addWidget(self.label)

        # UI → View signals
        self.edit.textChanged.connect(self.nameChanged.emit)

    # Presenter → View API
    def set_greeting(self, text: str) -> None:
        self.label.setText(text)


# -------------------------
# Presenter (glue)
# -------------------------

class GreetingPresenter(QObject):
    def __init__(self, model: GreetingModel, view: GreetingView) -> None:
        super().__init__()
        self.model = model
        self.view = view

        # View → Presenter
        self.view.nameChanged.connect(self.on_name_changed)

        # Initial render
        self.view.set_greeting(self.model.greeting())

    @pyqtSlot(str)
    def on_name_changed(self, text: str) -> None:
        self.model.name = text
        self.view.set_greeting(self.model.greeting())


# -------------------------
# Main
# -------------------------

def main() -> int:
    app = QApplication(sys.argv)

    model = GreetingModel()
    view = GreetingView()
    presenter = GreetingPresenter(model, view)  # keep reference!

    view.setWindowTitle("Minimal PyQt6 MVP")
    view.resize(300, 150)
    view.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
