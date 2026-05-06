import py_trees


class ReadTarget(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("ReadTarget")
        self.bb = self.attach_blackboard_client()
        self.bb.register_key("target", access=py_trees.common.Access.READ)

    def update(self):
        print(f"[ReadTarget] target = {self.bb.target}")
        return py_trees.common.Status.SUCCESS


# --- external data source (simulated) ---
counter = {"i": 0}


def pre_tick_handler(behaviour_tree):
    writer = py_trees.blackboard.Client(name="pre_tick_writer")
    writer.register_key("target", access=py_trees.common.Access.WRITE)

    i = counter["i"]
    writer.target = {"x": i, "y": i * 2}

    counter["i"] += 1


# --- build tree ---
root = py_trees.composites.Sequence(name="Root", memory=False)
root.add_child(ReadTarget())

tree = py_trees.trees.BehaviourTree(root)

# attach pre-tick
tree.add_pre_tick_handler(pre_tick_handler)

# run
for _ in range(5):
    tree.tick()