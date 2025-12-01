import numpy as np

def unit_quaternion(q):
    q = np.array(q, dtype=float)  # q = [w, x, y, z]
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Zero quaternion cannot be normalized")
    return q / norm

# Example
q = [2, 3, 1, 4]
q_unit = unit_quaternion(q)
print("Unit quaternion:", q_unit)
print("Norm:", np.linalg.norm(q_unit))  # should be 1
