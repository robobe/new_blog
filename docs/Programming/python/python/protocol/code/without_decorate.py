from typing import Protocol

class Speaker(Protocol):
    def speak(self) -> None: ...

class Dog:
    def speak(self): print("Woof!")

print(isinstance(Dog(), Speaker))  # ❌ TypeError
