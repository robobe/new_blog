---
title: MVP Model View Presenter
tags:
    - gui
    - mvp
---

MVP splits your UI code into three roles:

1️⃣ **Model — data & business logic**
- Holds application state
- Knows nothing about Qt or UI
- Example: data classes, domain logic, timers, IO, calculations

2️⃣ **View — UI only**
- Widgets, layouts, rendering
- Emits user intent (signals like clicked, textChanged)
- Has no business logic
- Does not pull data from Model

3️⃣ **Presenter — the glue**
- Listens to View events
- Updates the Model
- Listens to Model changes
- Pushes updates to the View

---

## Simple Demo

![](images/mvp.drawio.png)

```python
--8<-- "docs/Programming/GUI/QT/MVP/code/mvp_simple_demo.py"
```

### Model notify variant

![](images/mvp1.drawio.png)

```

```

### Brief remainder
Qt uses signals & slots to implement an event-driven, observer pattern.  
**pyqtSignal declares events; pyqtSlot receives them safely and efficiently.**

- **Signal** → “Something happened”
- **Slot** → “What should run when it happens”

#### **pyqtSignal**

- Declared at class level
- Signal type is defined by argument list
- Emits with .emit(...)
- One signal → many receivers


```python title="declare"
from PyQt6.QtCore import QObject, pyqtSignal

class GreetingView(QWidget):
    # User intent exposed as signals
    nameChanged = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()

        self.edit = QLineEdit()
        self.label = QLabel("Hello!")

        #...

        # UI → View signals
        self.edit.textChanged.connect(self.nameChanged.emit)
```

#### **pyqtSlot**
pyqtSlot marks a method as a Qt slot.

```python
from PyQt6.QtCore import pyqtSlot

class GreetingPresenter(QObject):
    @pyqtSlot(str)
    def on_name_changed(self, text: str) -> None:
        self.model.name = text
        self.view.set_greeting(self.model.greeting())

```
---

## Reference
- [Everything You NEED to Know About Client Architecture Patterns](https://youtu.be/I5c7fBgvkNY?t=122)