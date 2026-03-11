class Animal:
    def say_hello(self):
        raise NotImplementedError

class Dog(Animal):
    def say_hello(self):
        return "Hello from Dog!"

class Cat(Animal):
    def say_hello(self):
        return "Hello from Cat!"

def animal_factory(kind):
    if kind == "dog":
        return Dog()
    elif kind == "cat":
        return Cat()
    else:
        raise ValueError("Unknown animal")

animal = animal_factory("dog")
print(animal.say_hello())