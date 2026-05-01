# SKILL: PyQt MVP with Service + Pure Model + Qt Bridge

## Purpose

Generate a PyQt application using a strict MVP architecture with:

* Pure Python Model (no Qt)
* Service layer (data producers, threads, ROS, IO)
* Python event system (no Qt in model)
* Presenter as the ONLY Qt bridge
* Thread-safe UI updates via Qt signals

---

## Architecture Overview

```
Service (thread)
    ↓
Model (pure Python)
    ↓
Python Event (callback)
    ↓
Presenter (bridge)
    ↓
Qt Signal
    ↓
Qt Slot (main thread)
    ↓
View (UI)
```

---

## Rules

### 1. Model (STRICT)

* Must NOT import Qt
* Must NOT use pyqtSignal / QObject
* Must use pure Python (dataclass preferred)
* Must expose events using a custom Event class

Example:

```python
@dataclass
class Model:
    value: int = 0
    value_changed: Event = field(default_factory=Event)
```

---

### 2. Event System (REQUIRED)

Implement a simple Python callback system:

```python
class Event:
    def __init__(self):
        self._subs = []

    def subscribe(self, cb):
        self._subs.append(cb)

    def emit(self, value):
        for cb in self._subs:
            cb(value)
```

---

### 3. Service (STRICT)

* Must NOT import Qt
* Responsible for:

  * Threads / IO / ROS / external data
* Must update model only
* Must NOT touch View
* Must NOT emit Qt signals

Example:

```python
class Service:
    def __init__(self, model):
        self.model = model

    def run(self):
        self.model.set_value(new_value)
```

---

### 4. View (STRICT)

* Only Qt widgets
* No logic
* No model access
* Expose user actions via Qt signals

Example:

```python
class View(QWidget):
    clicked = pyqtSignal()

    def set_value(self, v):
        self.label.setText(str(v))
```

---

### 5. Presenter (CRITICAL COMPONENT)

Responsibilities:

* Connect View ↔ Service
* Subscribe to Model events (Python callbacks)
* Convert Python callbacks → Qt signals
* Update View ONLY inside Qt slot

Pattern:

```python
class Presenter(QObject):
    model_signal = pyqtSignal(int)

    def __init__(self, model, service, view):
        super().__init__()

        self.model = model
        self.service = service
        self.view = view

        # Python callback
        self.model.value_changed.subscribe(self._on_model_changed)

        # Qt bridge
        self.model_signal.connect(self._on_model_changed_qt)

    def _on_model_changed(self, value):
        # MAY BE FROM ANY THREAD
        self.model_signal.emit(value)

    @pyqtSlot(int)
    def _on_model_changed_qt(self, value):
        # SAFE UI UPDATE
        self.view.set_value(value)
```

---

### 6. Thread Safety (MANDATORY RULE)

NEVER update UI from:

* service
* model
* python callback

ALWAYS use:

```
callback → pyqtSignal → pyqtSlot → UI
```

---

### 7. Composition Root

All objects must be created in one place:

```python
class App:
    def __init__(self):
        self.model = Model()
        self.service = Service(self.model)
        self.view = View()
        self.presenter = Presenter(self.model, self.service, self.view)
```

Rules:

* Keep references to presenters
* Do not create objects inside other classes implicitly

---

### 8. Data Flow Rules

* Service updates Model
* Model emits Python event
* Presenter listens
* Presenter emits Qt signal
* Presenter updates View

NO shortcuts allowed.

---

### 9. Forbidden Patterns

❌ Model inherits QObject
❌ Service uses Qt
❌ View accesses Model directly
❌ Presenter polls Model
❌ UI updated from background thread

---

### 10. Extensions

When generating advanced code, support:

* multiple services
* multiple presenters
* shared model
* background threads
* ROS / sockets / file IO

---

## Output Requirements

When generating code:

* Always include:

  * Event class
  * Model
  * Service
  * View
  * Presenter
  * Main / composition root
* Keep files modular if possible
* Use clear naming: `XModel`, `XService`, `XView`, `XPresenter`

---

## Goal

Produce scalable, thread-safe UI architecture suitable for:

* robotics dashboards
* streaming data visualization
* real-time monitoring systems
