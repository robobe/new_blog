import numpy as np

def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def q_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])

# Rotation quaternion: 90° about Z
theta = np.pi/2
q = np.array([0, 0, np.sin(theta/2), np.cos(theta/2)])

# Conjugate (inverse)
q_conj = q_conjugate(q)

# Our vector (as pure quaternion)
v = np.array([1, 0, 0, 0])

# Rotate vector: v' = q * v * q^-1
v_rot = q_mult(q_mult(q, v), q_conj)

print("Original vector:", v[:3])
print("Rotated vector :", v_rot[:3])
