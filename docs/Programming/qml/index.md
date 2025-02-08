---
tags:
    - qt
    - qml
---

# QML

## Hello World

!!! note "uv"
    Using uv package manager

    ```bash
    pip install uv
    ```

    ```bash
    # create virtual env.
    uv venv

    # source
    source .venv/bin/activate

    # install qml
    uv install PySide6
    ```

     
### Projects

```bash
└── qml_tutorial
    ├── __init__.py
    ├── main.py
    └── main.qml
```

```python title="main.py"
import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtQml import QQmlApplicationEngine

# Create the application instance
app = QApplication(sys.argv)

# Load the QML file
engine = QQmlApplicationEngine()
engine.load("main.qml")

# Exit if QML fails to load
if not engine.rootObjects():
    sys.exit(-1)

# Run the application
sys.exit(app.exec())

```

```qml title="main.qml"
import QtQuick 6.5
import QtQuick.Controls 6.5

ApplicationWindow {
    visible: true
    width: 400
    height: 300
    title: "Hello QML with Python"

    Rectangle {
        anchors.fill: parent
        color: "lightblue"

        Text {
            text: "Hello, World!"
            anchors.centerIn: parent
            font.pixelSize: 24
        }
    }
}

```