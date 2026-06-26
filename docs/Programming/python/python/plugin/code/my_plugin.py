# /some/path/my_plugin.py

class MyPlugin:
    name = "my_plugin"

    def setup(self, config):
        self.factor = config.get("factor", 1)

    def process(self, data):
        return data * self.factor

# require every plugin file to expose a registration function
def register():
    return MyPlugin
