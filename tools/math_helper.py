import numpy as np
import matplotlib.pyplot as plt
import scipy.io

def print_norm(mat):
    print("norm: %.30f" % sum(np.array(mat).flatten()))


def euler2quat(yaw, pitch, roll):
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    q = np.array([
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr
    ])
    return q


def quat2euler(q):
    q0, q1, q2, q3 = q.T
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    pitch = np.arcsin(2 * (q0 * q2 - q3 * q1))
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
    return yaw, pitch, roll
