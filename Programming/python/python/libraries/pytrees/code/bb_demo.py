import py_trees
from py_trees.behaviour import Behaviour

# --- Action 1: Write to Blackboard ---
class WriteNumber(Behaviour):
    def __init__(self, name="WriteNumber"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="number", access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.number = 42
        print("[WriteNumber] Wrote number:", self.blackboard.number)
        return py_trees.common.Status.SUCCESS


# --- Action 2: Read from Blackboard ---
class ReadNumber(Behaviour):
    def __init__(self, name="ReadNumber"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key="number", access=py_trees.common.Access.READ)

    def update(self):
        number = self.blackboard.number
        print("[ReadNumber] Read number:", number)
        return py_trees.common.Status.SUCCESS


# --- Build Tree ---
root = py_trees.composites.Sequence(name="Sequence", memory=True)
root.add_children([
    WriteNumber(),
    ReadNumber(),
])

tree = py_trees.trees.BehaviourTree(root)

# --- Tick Tree ---
for i in range(3):
    print(f"\n--- Tick {i} ---")
    tree.tick()