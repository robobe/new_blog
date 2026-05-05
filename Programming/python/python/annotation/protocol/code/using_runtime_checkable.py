from typing import Protocol, runtime_checkable

@runtime_checkable
class Speaker(Protocol):
    def speak(self) -> None: ...

class Dog:
    def speak(self):
        print("Woof!")

d = Dog()
print(isinstance(d, Speaker))  # ✅ True
