import copy
import time
import threading
import py_trees


shared_data = {"target": None}
data_lock = threading.Lock()
stop_event = threading.Event()


def worker():
    i = 0
    while not stop_event.is_set():
        new_target = {
            "x": i,
            "y": i * 2,
        }

        with data_lock:
            shared_data["target"] = copy.deepcopy(new_target)

        i += 1
        time.sleep(0.2)


writer = py_trees.blackboard.Client(name="pre_tick_writer")
writer.register_key("target", access=py_trees.common.Access.WRITE)


def pre_tick_handler(tree):
    with data_lock:
        target_copy = copy.deepcopy(shared_data["target"])

    if target_copy is not None:
        writer.target = target_copy


class ReadTarget(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("ReadTarget")
        self.bb = self.attach_blackboard_client(name="reader")
        self.bb.register_key("target", access=py_trees.common.Access.READ)

    def update(self):
        try:
            target = self.bb.target
        except KeyError:
            print("[ReadTarget] no target yet")
            return py_trees.common.Status.FAILURE

        print("[ReadTarget] target:", target)
        return py_trees.common.Status.SUCCESS


root = py_trees.composites.Sequence(name="Root", memory=False)
root.add_child(ReadTarget())

tree = py_trees.trees.BehaviourTree(root)
tree.add_pre_tick_handler(pre_tick_handler)

thread = threading.Thread(target=worker, daemon=True)
thread.start()

try:
    while True:
        tree.tick()
        time.sleep(0.02)  # 50Hz
except KeyboardInterrupt:
    stop_event.set()
    thread.join()