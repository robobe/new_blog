class NonData:
    def __get__(self, instance, owner):
        return "descriptor value"


class A:
    x = NonData()
    y = NonData()


a = A()
a.x = "instance value"

print(a.x)   # "instance value" (descriptor ignored)
print(a.y)
print(a.__dict__)