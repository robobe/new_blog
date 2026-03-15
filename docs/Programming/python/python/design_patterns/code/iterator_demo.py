
class DataIterator:
    def __init__(self, data):
        self.data = data
        self.index = 0

    def __next__(self):
        if self.index >= len(self.data):
            raise StopIteration
        value = self.data[self.index]
        self.index += 1
        return value
    
class DataCollection:
    def __init__(self):
        self.data = list(range(1, 11))

    def __iter__(self):
        return DataIterator(self.data)
    
data = DataCollection()

iterator = iter(data)

print(next(iterator))
print(next(iterator))
print(next(iterator))