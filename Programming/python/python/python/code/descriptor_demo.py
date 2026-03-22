class MyDescriptor:
    def __get__(self, instance, owner):
        print("GET called")
        return 42

    def __set__(self, instance, value):
        print(f"SET called with {value}")


class MyClass:
    x = MyDescriptor()


obj = MyClass()

print(obj.x)   # triggers __get__
obj.x = 10     # triggers __set__