class Light:
    def turn_on(self):
        print("Light ON")

    def turn_off(self):
        print("Light OFF")

class LightOnCommand:
    def __init__(self, light):
        self.light = light

    def execute(self):
        self.light.turn_on()