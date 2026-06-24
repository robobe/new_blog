import numpy as np

dt = 1.0

# State: [position, velocity]
x = np.array([
    [0.0],
    [0.0]
])

F = np.array([
    [1.0, dt],
    [0.0, 1.0]
])

H = np.array([
    [1.0, 0.0]
])

P = np.array([
    [10.0, 0.0],
    [0.0, 10.0]
])

Q = np.array([
    [0.01, 0.0],
    [0.0, 0.1]
])

R = np.array([
    [4.0]
])

I = np.eye(2)

measurements = [1.2, 2.1, 2.9, 4.2, 5.0, 6.1]

for z_value in measurements:
    z = np.array([[z_value]])

    # --------------------
    # Predict
    # --------------------
    x_pred = F @ x
    P_pred = F @ P @ F.T + Q

    # --------------------
    # Correct
    # --------------------
    y = z - H @ x_pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    x = x_pred + K @ y
    P = (I - K @ H) @ P_pred

    print(f"measurement={z_value:.2f}, position={x[0,0]:.2f}, velocity={x[1,0]:.2f}")