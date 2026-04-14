import multiprocessing
from pyfranka.franka_pybind import *
import numpy as np
import math
import redis, msgpack
from threading import Thread, Lock
from multiprocessing import Process
from pyquaternion import Quaternion
import time


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


def rw_redis_data():
    global franka_state, set_tcp_speed, redis_obj
    while True:
        timestamp = time.time()
        redis_obj.write('GET_CURRENT_TIMESTAMPS', timestamp)

        redis_obj.write('GET_CURRENT_AXIS_EXTERNAL_TORQUE',
                        franka_state.tau_ext_hat_filtered)
        redis_obj.write('GET_CURRENT_MEASURED_TORQUE', franka_state.tau_J)
        redis_obj.write('GET_CURRENT_WRENCH', franka_state.O_F_ext_hat_K)
        redis_obj.write('GET_REALTIME_JOINT_SPEED', franka_state.dq)
        redis_obj.write('GET_REALTIME_JOINT_ACCELERATION', franka_state.ddq_d)
        redis_obj.write('GET_REALTIME_TCP_POSITION',
                        matrix2q(franka_state.O_T_EE))
        time.sleep(1e-3)
        # print('Redis cost', time.time() - timestamp)

        # tcp = franka_state.O_T_EE
        # redis_obj.write(
        #     'GET_REALTIME_TCP_POSITION',
        #     tcp[12:15] + Quaternion(matrix=np.array(tcp).reshape(4, 4).T,
        #                             atol=1e-4).q.tolist())


franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")
franka_api_obj.set_default_behavior()
franka_state = franka_api_obj.readOnce()

redis_obj = RedisBase('127.0.0.1', 6, 1)
# thread_obj = Thread(target=rw_redis_data)
# thread_obj.start()
process = multiprocessing.Process(target=rw_redis_data)
process.start()

# # First move the robot to a suitable joint configuration
# q_goal = [
#     0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4
# ]
# motion_generator = MotionGenerator(0.1, q_goal)
# franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)
# print("Finished moving to initial joint configuration.")

# Set additional parameters always before the control loop, NEVER in the control loop!
# Set the joint impedance.
franka_api_obj.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000])

# Set the collision behavior.
lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.]
upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
lower_torque_thresholds_acceleration = [
    25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0
]
upper_torque_thresholds_acceleration = [
    35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0
]

lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

franka_api_obj.set_collision_behavior(
    lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
    lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
    lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
    lower_force_thresholds_nominal, upper_force_thresholds_nominal)

time_max = 4.0
v_max = 0.1
angle = math.pi / 4.0
time_control = 0.0
speed_target = franka_state.O_dP_EE_d
set_tcp_speed = franka_state.O_dP_EE_d
T = 0.02
lock = Lock()


def cartesian_velocity_control(robot_state, period):
    global time_max, v_max, angle, time_control
    global franka_state, set_tcp_speed, speed_target, T
    global redis_obj, lock

    # timestamp = time.time()

    lock.acquire()
    franka_state = robot_state
    lock.release()
    time_control += period.toSec()

    acc_factor = 0.1
    max_ddP_EE = np.array([1.7, 1.7, 1.7, 2.5, 2.5, 2.5])
    max_ddP_EE = max_ddP_EE * acc_factor

    current_tcp_speed = robot_state.O_dP_EE_d

    set_tcp_speed = redis_obj.read("SET_REFERENCE_TCP_VELOCITY")
    if speed_target != set_tcp_speed:
        speed_target = set_tcp_speed
        time_control = period.toSec()
        T = max(
            abs((np.array(set_tcp_speed) - np.array(current_tcp_speed)) /
                max_ddP_EE))
        T = max(T, 20e-3)

    if time_control < T:
        output = np.array(current_tcp_speed) + time_control / T * (
            np.array(speed_target) - np.array(current_tcp_speed))
    else:
        output = np.array(speed_target)

    cartesian_velocities_obj = CartesianVelocities(output)

    # timestamp = time.time()
    # redis_obj.write('GET_CURRENT_TIMESTAMPS', timestamp)
    # redis_obj.write('GET_CURRENT_AXIS_EXTERNAL_TORQUE',
    #                 franka_state.tau_ext_hat_filtered)
    # redis_obj.write('GET_CURRENT_MEASURED_TORQUE', franka_state.tau_J)
    # redis_obj.write('GET_CURRENT_WRENCH', franka_state.O_F_ext_hat_K)
    # redis_obj.write('GET_REALTIME_JOINT_SPEED', franka_state.dq)
    # redis_obj.write('GET_REALTIME_JOINT_ACCELERATION', franka_state.ddq_d)
    # redis_obj.write('GET_REALTIME_TCP_POSITION', matrix2q(franka_state.O_T_EE))
    # print('Control cost', time.time() - timestamp)
    return cartesian_velocities_obj


franka_api_obj.robot_control(
    cartesian_velocities_handle=cartesian_velocity_control)
