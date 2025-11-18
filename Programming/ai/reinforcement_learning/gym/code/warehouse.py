# https://youtu.be/AoGRjPt-vms

import random
from enum import Enum


# Actions the Robot is capable of performing i.e. go in a certain direction
class RobotAction(Enum):
    LEFT = 0
    DOWN = 1
    RIGHT = 2
    UP = 3


# The Warehouse is divided into a grid. Use these 'tiles' to represent the objects on the grid.
class GridTile(Enum):
    _FLOOR = 0
    ROBOT = 1
    TARGET = 2

    # Return the first letter of tile name, for printing to the console.
    def __str__(self):
        return self.name[:1]


class WarehouseRobot:
    def __init__(self, grid_rows=5, grid_cols=5):
        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.reset()

    def reset(self, seed=None):
        """place the robot at 0,0 and the robot at random place

        """
        self.robot_pos = [0, 0]
        random.seed(seed)
        self.target_pos = [
            random.randint(1, self.grid_rows - 1),
            random.randint(1, self.grid_cols - 1),
        ]

    def perform_action(self, robot_action: RobotAction) -> bool:
        """move the robot

        Args:
            robot_action (RobotAction): robot action

        Returns:
            bool: True if robot reached target
        """
        if robot_action == RobotAction.LEFT:
            self.robot_pos[0] = max(0, self.robot_pos[0] - 1)
        elif robot_action == RobotAction.RIGHT:
            self.robot_pos[0] = min(self.grid_rows - 1, self.robot_pos[0] + 1)
        elif robot_action == RobotAction.UP:
            self.robot_pos[1] = max(0, self.robot_pos[1] - 1)
        elif robot_action == RobotAction.DOWN:
            self.robot_pos[1] = min(self.grid_cols - 1, self.robot_pos[1] + 1)

        return self.robot_pos == self.target_pos

    

    def render(self):
        """render board
        """
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                if [r, c] == self.robot_pos:
                    print(str(GridTile.ROBOT), end=" ")
                elif [r, c] == self.target_pos:
                    print(str(GridTile.TARGET), end=" ")
                else:
                    print(str(GridTile._FLOOR), end=" ")
                
            print()
        print()

if __name__ == "__main__":
    warehouse_robot = WarehouseRobot(grid_rows=5, grid_cols=5)
    warehouse_robot.render()
    
    for i in range(10):
        action = random.choice(list(RobotAction))
        print(action)
        
        warehouse_robot.perform_action(action)
        warehouse_robot.render()