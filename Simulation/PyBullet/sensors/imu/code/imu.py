import time
import numpy as np
import pybullet as p
import pybullet_data


class SimIMU:
    def __init__(self, body_id, dt):
        self.body_id = body_id
        self.dt = dt
        self.last_lin_vel = np.zeros(3)

        self.gravity = np.array([0.0, 0.0, -9.81])

        # simple MPU6050-like imperfection
        self.acc_bias = np.array([0.02, -0.01, 0.03])
        self.gyro_bias = np.array([0.001, -0.002, 0.001])

        self.acc_noise_std = 0.05      # m/s^2
        self.gyro_noise_std = 0.002    # rad/s

    def read(self):
        pos, orn = p.getBasePositionAndOrientation(self.body_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.body_id)

        lin_vel = np.array(lin_vel)
        ang_vel = np.array(ang_vel)

        acc_world = (lin_vel - self.last_lin_vel) / self.dt
        self.last_lin_vel = lin_vel.copy()

        # accelerometer measures specific force
        specific_force_world = acc_world - self.gravity

        # world -> body frame
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        acc_body = R.T @ specific_force_world
        gyro_body = R.T @ ang_vel

        # add noise + bias
        acc_body += self.acc_bias + np.random.normal(0, self.acc_noise_std, 3)
        gyro_body += self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)

        return acc_body, gyro_body


def main():
    dt = 1.0 / 240.0

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)

    p.loadURDF("plane.urdf")

    # Create moving box
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.25, 0.25, 0.25],
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.25, 0.25, 0.25],
        rgbaColor=[0.1, 0.4, 1.0, 1.0],
    )

    box_id = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0, 0, 1.0],
    )

    imu = SimIMU(box_id, dt)

    step = 0

    while True:
        t = step * dt

        # Move object using external force
        fx = 5.0 * np.sin(2.0 * np.pi * 0.5 * t)
        fy = 3.0 * np.cos(2.0 * np.pi * 0.25 * t)

        p.applyExternalForce(
            objectUniqueId=box_id,
            linkIndex=-1,
            forceObj=[fx, fy, 0],
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME,
        )

        # Add torque so gyro will also change
        p.applyExternalTorque(
            objectUniqueId=box_id,
            linkIndex=-1,
            torqueObj=[0.2, 0.1, 0.5],
            flags=p.LINK_FRAME,
        )

        p.stepSimulation()

        acc, gyro = imu.read()

        if step % 24 == 0:  # print about 10 Hz
            pos, orn = p.getBasePositionAndOrientation(box_id)

            print("------")
            print(f"time: {t:.2f} sec")
            print(f"pos:  {np.round(pos, 3)}")
            print(f"acc:  {np.round(acc, 3)} m/s^2")
            print(f"gyro: {np.round(gyro, 3)} rad/s")

        step += 1
        time.sleep(dt)


if __name__ == "__main__":
    main()