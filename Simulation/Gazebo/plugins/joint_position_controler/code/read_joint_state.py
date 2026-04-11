import math
import threading
import time

from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model

q = 0.0
qdot = 0.0
have_state = False
lock = threading.Lock()

q_ref = 0.7
k1 = 12.0
k2 = 3.5
tau_max = 8.0

def on_joint_state(msg: Model):
    global q, qdot, have_state
    print(msg)
    for joint in msg.joint:
        if joint.name != "joint1":
            continue

        pos = joint.axis1.position if joint.axis1.HasField("position") else 0.0
        vel = joint.axis1.velocity if joint.axis1.HasField("velocity") else 0.0

        with lock:
            q = pos
            qdot = vel
            have_state = True
        return

node = Node()
ok = node.subscribe(Model, "/world/default/model/joint_position_controller_demo/joint_state", on_joint_state)
if not ok:
    raise RuntimeError("Failed to subscribe to /simple_arm/joint_state")

pub_topic = "/model/simple_arm/joint/joint1/cmd_force"

while True:
    with lock:
        local_have = have_state
        local_q = q
        local_qdot = qdot

    if local_have:
        e = local_q - q_ref
        tau = -(k1 * e + k2 * local_qdot)
        tau = max(-tau_max, min(tau_max, tau))

        msg = Double()
        msg.data = tau
        # node.publish(pub_topic, msg)

        print(f"q={local_q:.3f}, qdot={local_qdot:.3f}, tau={tau:.3f}")

    time.sleep(0.005)