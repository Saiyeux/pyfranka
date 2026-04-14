import math
import numpy as np


def matrix2q(T):
    tr = T[0] + T[5] + T[10]
    if (tr > 0):
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (T[6] - T[9]) / S
        qy = (T[8] - T[2]) / S
        qz = (T[1] - T[4]) / S
    elif ((T[0] > T[5]) & (T[0] > T[10])):
        S = math.sqrt(1.0 + T[0] - T[5] - T[10]) * 2
        qw = (T[6] - T[9]) / S
        qx = 0.25 * S
        qy = (T[1] + T[4]) / S
        qz = (T[2] + T[8]) / S
    elif (T[5] > T[8]):
        S = math.sqrt(1.0 + T[5] - T[0] - T[10]) * 2
        qw = (T[2] - T[8]) / S
        qx = (T[1] + T[4]) / S
        qy = 0.25 * S
        qz = (T[6] + T[9]) / S
    else:
        S = math.sqrt(1.0 + T[10] - T[0] - T[5]) * 2
        qw = (T[1] - T[4]) / S
        qx = (T[2] + T[8]) / S
        qy = (T[6] + T[9]) / S
        qz = 0.25 * S
    return T[12], T[13], T[14], qw, qx, qy, qz


def q2matrix(q):
    m = [0 for x in range(0, 16)]
    sqw = q[3] * q[3]
    sqx = q[4] * q[4]
    sqy = q[5] * q[5]
    sqz = q[6] * q[6]
    m[0] = sqx - sqy - sqz + sqw
    m[5] = -sqx + sqy - sqz + sqw
    m[10] = -sqx - sqy + sqz + sqw

    tmp1 = q[4] * q[5]
    tmp2 = q[6] * q[3]
    m[1] = 2.0 * (tmp1 + tmp2)
    m[4] = 2.0 * (tmp1 - tmp2)

    tmp1 = q[4] * q[6]
    tmp2 = q[5] * q[3]
    m[2] = 2.0 * (tmp1 - tmp2)
    m[8] = 2.0 * (tmp1 + tmp2)

    tmp1 = q[5] * q[6]
    tmp2 = q[4] * q[3]
    m[6] = 2.0 * (tmp1 + tmp2)
    m[9] = 2.0 * (tmp1 - tmp2)

    m[12:15] = q[0:3]
    m[3] = m[7] = m[11] = 0.0
    m[15] = 1.0
    return m


def skew_symmetric_matrix(vector3d):
    return np.array([[0, -vector3d[2], vector3d[1]],
                     [vector3d[2], 0, -vector3d[0]],
                     [-vector3d[1], vector3d[0], 0]]).reshape(3, 3)


def linear_interpolation(t, v0, v1):
    return v0 + t * (v1 - v0) if (t < 1) else v1


def polynomial_interpolation(t, T, q0, q1, v0, v1, acc0, acc1):
    h = q1 - q0
    a0 = q0
    a1 = v0
    a2 = acc0 / 2
    a3 = (20 * h - (8 * v1 + 12 * v0) * T -
          (3 * acc0 - acc1) * T**2) / (2 * T**3)
    a4 = (-30 * h + (14 * v1 + 16 * v0) * T +
          (3 * acc0 - 2 * acc1) * T**2) / (2 * T**4)
    a5 = (12 * h - 6 * (v1 + v0) * T + (acc1 - acc0) * T**2) / (2 * T**5)
    return (a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 +
            a5 * t**5 if t < T else q1)
