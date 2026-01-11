from collections.abc import Callable

def func(callback: Callable[[int], str]) -> str:
    return callback(10)

def foo(data: int) -> str:
    return f"Hello, your number is {data}"

print(func(foo))
