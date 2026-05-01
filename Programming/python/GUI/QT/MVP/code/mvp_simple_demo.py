"""
PyQt6 MVP example

Service -> Model -> pure Python event -> Presenter -> Qt signal -> View

Run:
  pip install PyQt6
  python app.py
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget


# -------------------------
# Pure Python Event
# -------------------------

class Event:
    def __init__(self) -> None:
        self._subscribers: list[Callable[[int], None]] = []

    def subscribe(self, callback: Callable[[int], None]) -> None:
        self._subscribers.append(callback)

    def fire(self, value: int) -> None:
        for callback in self._subscribers:
            callback(value)


# -------------------------
# Model: pure Python, no Qt
# -------------------------

@dataclass
class CounterModel:
    value: int = 0
    value_changed: Event = field(default_factory=Event)

    def set_value(self, value: int) -> None:
        if self.value == value:
            return

        self.value = value
        self.value_changed.fire(value)


# -------------------------
# Service: pure Python, no Qt
# -------------------------

class NumberService:
    def __init__(self, model: CounterModel) -> None:
        self.model = model
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        counter = 0

        while self._running:
            counter += 1
            self.model.set_value(counter)
            time.sleep(0.5)


# -------------------------
# View: Qt only, no logic
# -------------------------

class CounterView(QWidget):
    start_clicked = pyqtSignal()
    stop_clicked = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()

        self.value_label = QLabel("Value: 0")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")

        layout = QVBoxLayout(self)
        layout.addWidget(self.value_label)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)

        self.start_button.clicked.connect(self.start_clicked.emit)
        self.stop_button.clicked.connect(self.stop_clicked.emit)

    def set_value(self, value: int) -> None:
        self.value_label.setText(f"Value: {value}")

    def set_running(self, running: bool) -> None:
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)


# -------------------------
# Presenter: Qt boundary
# -------------------------

class CounterPresenter(QObject):
    model_value_changed = pyqtSignal(int)

    def __init__(
        self,
        model: CounterModel,
        service: NumberService,
        view: CounterView,
    ) -> None:
        super().__init__()

        self.model = model
        self.service = service
        self.view = view

        # View -> Presenter
        self.view.start_clicked.connect(self.on_start_clicked)
        self.view.stop_clicked.connect(self.on_stop_clicked)

        # Model pure Python callback -> Qt signal
        self.model.value_changed.subscribe(self.on_model_value_changed)

        # Qt signal -> Qt slot
        self.model_value_changed.connect(self.on_model_value_changed_qt)

        self.render()
        self.view.set_running(False)

    def render(self) -> None:
        self.view.set_value(self.model.value)

    def on_model_value_changed(self, value: int) -> None:
        """
        Pure Python callback.

        This may be called from the service thread.
        Do NOT update the Qt UI here.
        """
        self.model_value_changed.emit(value)

    @pyqtSlot(int)
    def on_model_value_changed_qt(self, value: int) -> None:
        """
        Qt slot.

        This runs safely in the Qt main thread.
        """
        self.view.set_value(value)

    @pyqtSlot()
    def on_start_clicked(self) -> None:
        self.service.start()
        self.view.set_running(True)

    @pyqtSlot()
    def on_stop_clicked(self) -> None:
        self.service.stop()
        self.view.set_running(False)


# -------------------------
# Composition Root
# -------------------------

class AppCompositionRoot:
    def __init__(self) -> None:
        self.model = CounterModel()
        self.service = NumberService(self.model)
        self.view = CounterView()

        self.presenter = CounterPresenter(
            model=self.model,
            service=self.service,
            view=self.view,
        )

    def show(self) -> None:
        self.view.setWindowTitle("Service -> Model -> Presenter -> View")
        self.view.resize(300, 120)
        self.view.show()


# -------------------------
# Main
# -------------------------

def main() -> int:
    app = QApplication(sys.argv)

    composition = AppCompositionRoot()
    composition.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())