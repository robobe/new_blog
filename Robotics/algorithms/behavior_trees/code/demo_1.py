from time import sleep
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.trees import BehaviourTree
from py_trees import logging as log_tree


class Action(Behaviour):
  def __init__(self, name):
    super(Action, self).__init__(name)

  def setup(self):
    self.logger.debug(f"Action::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Action::initialise {self.name}")

  def update(self):
    self.logger.debug(f"Action::update {self.name}")
    sleep(1)
    return Status.RUNNING

  def terminate(self, new_status):
    self.logger.debug(f"Action::terminate {self.name} to {new_status}")



def make_bt():
  root = Sequence(name="sequence", memory=True)

  action_1 = Action("action 1")
  action_2 = Action("action 2")
  action_3 = Action("action 3")

  root.add_children(
      [
          action_1,
          action_2,
          action_3
      ]
  )

  return root


if __name__ == "__main__":
  log_tree.level = log_tree.Level.DEBUG
  root = make_bt()
  
  tree = BehaviourTree(root)
  tree.setup(timeout=15.0)

  for _ in range(5):
    tree.tick()
    time.sleep(0.5)
