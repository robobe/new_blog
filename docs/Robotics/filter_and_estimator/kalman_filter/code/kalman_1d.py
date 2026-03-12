import numpy as np
import matplotlib.pyplot as plt

true_value = 72
error_meas = 4
init_estimate = 68
error_estimate = 2
measurements = [75, 71, 70, 74]
estimates = []
for measurement in measurements:
    # Kalman Gain
    kalman_gain = error_estimate / (error_estimate + error_meas)

    # Update estimate with measurement
    estimate = init_estimate + kalman_gain * (measurement - init_estimate)

    # Update error estimate
    error_estimate = (1 - kalman_gain) * error_estimate

    print(f"Measurement: {measurement}, Estimate: {estimate:.2f}, Error Estimate: {error_estimate:.2f}")

    # Prepare for next iteration
    init_estimate = estimate
    estimates.append(estimate)

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(measurements, label='Measurements', marker='o')
plt.plot(estimates, label='Kalman Estimates', marker='x')
plt.axhline(true_value, color='r', linestyle='--', label='True Value')
plt.title('Kalman Filter 1D Estimation')
plt.xlabel('Measurement Index')
plt.ylabel('Value')
plt.legend()
plt.grid()
plt.show()