from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from typing import Optional
from urllib import error as url_error
from urllib import parse as url_parse
from urllib import request as url_request

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
)

from plugin_api import PluginHandle

PLUGIN_ID = "led-off"
PLUGIN_NAME = "LED OFF"


# -------------------------
# Model (Qt-free)
# -------------------------

@dataclass
class PluginModel:
    base_url: str = "http://127.0.0.1:8000"
    name: str = ""
    greeting: str = ""
    error: str = ""
    loading: bool = False

    def gpio(self, name: str, value: bool) -> str:
        # Build query string
        query = url_parse.urlencode(
            {"name": name, "value": "true" if value else "false"}
        )

        url = f"{self.base_url}/gpio?{query}"

        # POST request with NO body
        req = url_request.Request(
            url=url,
            method="POST",
            headers={
                "Accept": "application/json",
            },
        )

        try:
            with url_request.urlopen(req, timeout=5) as resp:
                if resp.status != 200:
                    raise RuntimeError(f"HTTP {resp.status}")
                payload = json.loads(resp.read().decode("utf-8"))
        except url_error.URLError as exc:
            raise RuntimeError(f"Request failed: {exc}") from exc

        greeting = payload.get("greeting")
        if not isinstance(greeting, str):
            raise RuntimeError("Invalid response payload")

        return greeting


# -------------------------
# View (UI only)
# -------------------------

class PluginView(QWidget):
    nameChanged = pyqtSignal(str)
    baseUrlChanged = pyqtSignal(str)
    fetchClicked = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        self.edit_base_url = QLineEdit()
        self.edit_name = QLineEdit()
        self.btn_fetch = QPushButton("LED off")
        self.lbl_status = QLabel()
        self.lbl_greeting = QLabel()

        root = QVBoxLayout(self)

        

        root.addWidget(self.btn_fetch)


        self.edit_name.textChanged.connect(self.nameChanged.emit)
        self.edit_base_url.textChanged.connect(self.baseUrlChanged.emit)
        self.btn_fetch.clicked.connect(self.fetchClicked.emit)

    def set_greeting(self, text: str) -> None:
        self.lbl_greeting.setText(text)

    def set_status(self, text: str) -> None:
        self.lbl_status.setText(text)

    def set_loading(self, loading: bool) -> None:
        self.btn_fetch.setEnabled(not loading)


# -------------------------
# Presenter (glue)
# -------------------------

class PluginPresenter(QObject):
    resultReady = pyqtSignal(str)
    errorReady = pyqtSignal(str)
    loadingChanged = pyqtSignal(bool)

    def __init__(self, model: PluginModel, view: PluginView) -> None:
        super().__init__()
        self.model = model
        self.view = view
        self._worker: Optional[threading.Thread] = None

        self.view.fetchClicked.connect(self.on_fetch_clicked)

        self.resultReady.connect(self.on_result)
        self.errorReady.connect(self.on_error)
        self.loadingChanged.connect(self.on_loading_changed)

        self.render()

    def render(self) -> None:
        self.view.set_greeting(self.model.greeting)
        if self.model.loading:
            self.view.set_status("Loading...")
            self.view.set_loading(True)
        else:
            self.view.set_status(self.model.error)
            self.view.set_loading(False)



    @pyqtSlot()
    def on_fetch_clicked(self) -> None:
        if self.model.loading:
            return
        self.model.error = ""
        self.model.greeting = ""
        self.model.loading = True
        self.render()

        name = "GPA0"

        def run() -> None:
            try:
                greeting = self.model.gpio(name, False)
                self.resultReady.emit(greeting)
            except Exception as exc:
                self.errorReady.emit(str(exc))
            finally:
                self.loadingChanged.emit(False)

        self._worker = threading.Thread(target=run, daemon=True)
        self._worker.start()

    @pyqtSlot(str)
    def on_result(self, greeting: str) -> None:
        self.model.greeting = greeting
        self.model.error = ""
        self.render()

    @pyqtSlot(str)
    def on_error(self, message: str) -> None:
        self.model.error = message
        self.model.greeting = ""
        self.render()

    @pyqtSlot(bool)
    def on_loading_changed(self, loading: bool) -> None:
        self.model.loading = loading
        self.render()

    def dispose(self) -> None:
        self.model.loading = False


# -------------------------
# Plugin factory
# -------------------------

def create_plugin(parent: Optional[QWidget] = None) -> PluginHandle:
    model = PluginModel()
    view = PluginView(parent)
    presenter = PluginPresenter(model, view)
    return PluginHandle(
        plugin_id=PLUGIN_ID,
        name=PLUGIN_NAME,
        widget=view,
        presenter=presenter,
        dispose=presenter.dispose,
    )
