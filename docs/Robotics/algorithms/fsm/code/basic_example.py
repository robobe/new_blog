from transitions import Machine

class Robot:
    pass


robot = Robot()

states = [
    "IDLE",
    "TRACKING",
    "ERROR"
]

machine = Machine(
    model=robot,
    states=states,
    initial="IDLE"
)

machine.add_transition(
    trigger="start_tracking",
    source="IDLE",
    dest="TRACKING"
)

machine.add_transition(
    trigger="error",
    source="*",
    dest="ERROR"
)

machine.add_transition(
    trigger="reset",
    source="ERROR",
    dest="IDLE"
)

print(robot.state)
# call trigger
robot.start_tracking()
# print state
print(robot.state)
# call trigger
robot.error()
print(robot.state)
# call trigger
robot.reset()
print(robot.state)