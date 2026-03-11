class DataProcessor:

    def run(self):
        self.load()
        self.process()
        self.save()

    def load(self):
        print("Loading data")

    def process(self):
        raise NotImplementedError()

    def save(self):
        print("Saving results")

class ImageProcessor(DataProcessor):

    def process(self):
        print("Processing image")

class TextProcessor(DataProcessor):

    def process(self):
        print("Processing text")

img = ImageProcessor()
img.run()

print("--" * 20)
txt = TextProcessor()
txt.run()