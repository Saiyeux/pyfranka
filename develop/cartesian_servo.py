from numpy.lib.function_base import append
from pyquaternion.quaternion import Quaternion
from pyfranka.franka_pybind import *
import sys
import math
import time
import copy

import redis
import msgpack
import numpy as np
from threading import Thread


class RedisBase():
    def __init__(self, host, database, socket_connect_timeout):
        self.__r = redis.StrictRedis(
            host=host,
            port=6379,
            db=database,
            socket_connect_timeout=socket_connect_timeout)

    def write(self, key, value):
        try:
            self.__r.set(key.encode('utf-8'), msgpack.dumps(value))
        except Exception as e:
            print(Exception, e)
            return None

    def read(self, key):
        try:
            return msgpack.unpackb(self.__r.get(key.encode('utf-8')))
        except Exception as e:
            print(Exception, e)
            print("data is not existed")
            return None


def rw_redis_data():
    global set_tcp_position, current_joint_position, current_tcp_position
    while True:
        # t1 = time.time()
        set_tcp_position = redis_obj.read("SET_REFERENCE_TCP_POSITION")
        # redis_obj.write("GET_REALTIME_JOINT_POSITION", current_joint_position)
        redis_obj.write("GET_REALTIME_TCP_POSITION", current_tcp_position)


def calculateDesiredValues(t, a0, a1, a2, a3, a4, a5):
    if t < T:
        p_d = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    else:
        p_d = p_goal
    return p_d


def polynomial(p0, p1, v0, v1, acc0, acc1):
    speed_factor = 0.05
    dp_max = np.array([1., 1., 1., 2., 2., 2., 2.]) * speed_factor  # m/s
    kDeltaQMotionFinished = 1e-6

    h = p1 - p0
    T = max(abs(h) / dp_max)
    T = max(T, 2000e-3)

    a0 = p0
    a1 = v0
    a2 = acc0 / 2
    a3 = (20 * h - (8 * v1 + 12 * v0) * T -
          (3 * acc0 - acc1) * T**2) / (2 * T**3)
    a4 = (-30 * h + (14 * v1 + 16 * v0) * T +
          (3 * acc0 - 2 * acc1) * T**2) / (2 * T**4)
    a5 = (12 * h - 6 * (v1 + v0) * T + (acc1 - acc0) * T**2) / (2 * T**5)
    return T, a0, a1, a2, a3, a4, a5


time_count = 0.0
q_goal = [
    0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4
]
motion_generator = MotionGenerator(0.1, q_goal)
p_d = list()
p_goal = np.zeros(7)
p_start = np.array([0., 0., 0., 1., 0., 0., 0.])
T = 0.
current_tcp_position = [0., 0., 0., 1., 0., 0., 0.]
set_tcp_position = [0., 0., 0., 1., 0., 0., 0.]
a0 = a1 = a2 = a3 = a4 = a5 = np.zeros(7)


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


def skew_symmetric_matrix(vector4d):
    return np.array([[0, -vector4d[0], -vector4d[1], -vector4d[2]],
                     [vector4d[0], 0, vector4d[2], -vector4d[1]],
                     [vector4d[1], -vector4d[2], 0, vector4d[0]],
                     [vector4d[2], vector4d[1], -vector4d[0],
                      0]]).reshape(4, 4)


# def skew_symmetric_matrix(vector4d):
#     return np.array([[0, -vector4d[0], -vector4d[1], -vector4d[2]],
#                      [vector4d[0], 0, -vector4d[2], vector4d[1]],
#                      [vector4d[1], vector4d[2], 0, -vector4d[0]],
#                      [vector4d[2], -vector4d[1], vector4d[0],
#                       0]]).reshape(4, 4)

filename = './data/data.csv'
of = open(filename, 'w')

header = [
    'time', 'x', 'y', 'z', 'w', 'i', 'j', 'k', 'dx', 'dy', 'dz', 'dwx', 'dwy',
    'dwz', 'ddx', 'ddy', 'ddz', 'ddwx', 'ddwy', 'ddwz'
]
of.writelines(','.join(header) + ',')
of.write('\n')


def cartesian_servo_control(robot_state, period):
    # time_start = time.time()
    global time_count
    global p_d, p_start, p_goal
    global current_tcp_position, set_tcp_position
    global a0, a1, a2, a3, a4, a5, T

    time_count = time_count + period.toSec()
    current_tcp_position = matrix2q(robot_state.O_T_EE_c)

    if (p_d == []):
        p_d = current_tcp_position

    if (p_goal == np.zeros(7)).all():
        p_goal = np.array(current_tcp_position)

    if (np.array(set_tcp_position) != p_goal).any():
        p_start = np.array(current_tcp_position)
        p_goal = np.array(set_tcp_position)
        dp = robot_state.O_dP_EE_c
        dp_start = np.append(
            dp[:3], 0.5 * p_start[3:] @ skew_symmetric_matrix(dp[3:]).T)
        # dp_start = np.zeros(7)
        dp_goal = np.zeros(7)
        ddp = robot_state.O_ddP_EE_c
        ddp_start = np.append(
            ddp[:3], 0.5 * p_start[3:]
            @ (skew_symmetric_matrix(ddp[3:]).T + skew_symmetric_matrix(
                dp[3:]).T @ skew_symmetric_matrix(dp[3:]).T))
        # ddp_start = np.zeros(7)

        ddp_goal = np.zeros(7)
        T, a0, a1, a2, a3, a4, a5 = polynomial(p_start, p_goal, dp_start,
                                               dp_goal, ddp_start, ddp_goal)
        time_count = period.toSec()
        # print(T, a0, a1, a2, a3, a4, a5)

    p_d = calculateDesiredValues(time_count, a0, a1, a2, a3, a4, a5)

    # of.write(str(time_count) + ',')
    # for x in p_d:
    #     of.write(str(x))
    #     of.write(',')
    # for x in robot_state.O_dP_EE_c:
    #     of.write(str(x))
    #     of.write(',')
    # for x in robot_state.O_ddP_EE_c:
    #     of.write(str(x))
    #     of.write(',')
    # of.write('\n')

    # m = robot_state.O_T_EE_c
    # m[12:15] = q2matrix(p_d)[12:15]
    m = q2matrix(p_d)
    cartesian_pose_obj = CartesianPose(m)
    # cartesian_pose_obj = CartesianPose(robot_state.O_T_EE_c)
    # print(time.time() - time_start)
    return cartesian_pose_obj


redis_obj = RedisBase("localhost", 6, 1)
thread_obj = Thread(target=rw_redis_data)
thread_obj.start()

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")
franka_api_obj.set_default_behavior()
franka_api_obj.set_collision_behavior()
error_count = 0

while True:
    time_count = 0.0
    if franka_api_obj.robot_control(
            cartesian_pose_handle=cartesian_servo_control):
        error_count = error_count + 1
        franka_api_obj.set_default_behavior()
        time.sleep(2)
        print('auto recover from errors')
