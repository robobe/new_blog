from enum import Enum

from transitions import Machine


class RobotState(Enum):
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    ERROR = "ERROR"


class Robot:
    def on_enter_ERROR(self):
        print("----enter error")

    def on_exit_ERROR(self):
        print("----exit error")

    

robot = Robot()

states = [
    RobotState.IDLE,
    RobotState.TRACKING,
    RobotState.ERROR
]

machine = Machine(
    model=robot,
    states=states,
    initial=RobotState.IDLE
)

machine.add_transition(
    trigger="start_tracking",
    source=RobotState.IDLE,
    dest=RobotState.TRACKING
)

machine.add_transition(
    trigger="error",
    source="*",
    dest=RobotState.ERROR
)

machine.add_transition(
    trigger="reset",
    source=RobotState.ERROR,
    dest=RobotState.IDLE
)

print(robot.state)
# call trigger
robot.start_tracking()
# print state
print(robot.state)
# call trigger
robot.error()
# check if in state
if robot.is_ERROR():
    print("**robot has error**")
# print state
print(robot.state)
# call trigger
robot.reset()
print(robot.state)
# move to state without trigger
robot.to_ERROR()
print(robot.state)
