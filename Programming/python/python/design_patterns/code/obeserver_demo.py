class Subject:
    def __init__(self):
        self.observers = []

    def subscribe(self, observer):
        self.observers.append(observer)

    def notify(self, message):
        for observer in self.observers:
            observer.update(message)

class Observer:
    def update(self, message):
        print("Received:", message)

subject = Subject()
observer1 = Observer()
observer2 = Observer()

subject.subscribe(observer1)
subject.subscribe(observer2)
subject.notify("Hello Observers!")