from pyfranka.franka_pybind import *
import math
import time
import numpy as np
import redis
import msgpack
from threading import Thread
from pyquaternion import Quaternion


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


current_tcp_matrix = current_tcp_q = list()
ext_tau = measured_tau = list()


def rw_redis_data():
    global jp, current_joint_position, current_tcp_position, ext_force, current_tcp_matrix, current_tcp_q, ext_tau, measured_tau
    global franka_state
    while True:
        # t1 = time.time()
        jp = redis_obj.read("SET_REFERENCE_JOINT_POSITION")
        redis_obj.write("GET_REALTIME_JOINT_POSITION", current_joint_position)
        redis_obj.write("GET_REALTIME_TCP_POSITION", current_tcp_position)
        redis_obj.write("GET_REALTIME_TCP_MATRIX", current_tcp_matrix)
        redis_obj.write("GET_REALTIME_TCP_Q", current_tcp_q)
        redis_obj.write("GET_CURRENT_TCP_EXTERNAL_FORCE_TORQUE",
                        ext_force.tolist())
        redis_obj.write('GET_CURRENT_AXIS_EXTERNAL_TORQUE', ext_tau)
        redis_obj.write('GET_CURRENT_MEASURED_TORQUE', measured_tau)
        redis_obj.write('GET_CURRENT_WRENCH', franka_state.O_F_ext_hat_K)
        redis_obj.write('GET_CURRENT_MASS_MATRIX', model.mass(franka_state))
        redis_obj.write('GET_CURRENT_GRAVITY',
                        model.gravity(franka_state, [0., 0., -9.81]))
        redis_obj.write('GET_CURRENT_CORIOLIS', model.coriolis(franka_state))
        redis_obj.write('GET_CURRENT_ZERO_JACOBIAN',
                        model.zeroJacobian(Frame.kEndEffector, franka_state))
        redis_obj.write('GET_CURRENT_BODY_JACOBIAN',
                        model.bodyJacobian(Frame.kEndEffector, franka_state))
        redis_obj.write('GET_REALTIME_JOINT_SPEED', franka_state.dq)
        redis_obj.write('GET_REALTIME_JOINT_ACCELERATION', franka_state.ddq_d)
        redis_obj.write('GET_CURRENT_CARTESIAN_CONTACT',
                        franka_state.cartesian_contact)
        redis_obj.write('GET_CURRENT_JOINT_CONTACT',
                        franka_state.joint_contact)


# t2 = time.time()
# print("redis thead", t2-t1)


def calculateDesiredValues(t, a0, a1, a2, a3, a4, a5):
    global T
    if t < T:
        q_d = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    else:
        q_d = q_goal
    return q_d


def polynomial(q0, q1, v0, v1, acc0, acc1):
    speed_factor = 0.1
    dq_max = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]) * speed_factor
    kDeltaQMotionFinished = 1e-6

    h = q1 - q0
    T = max(abs(h) / dq_max)
    T = max(T, 500e-3)

    if T > 0.:
        a0 = q0
        a1 = v0
        a2 = acc0 / 2
        a3 = (20 * h - (8 * v1 + 12 * v0) * T -
              (3 * acc0 - acc1) * T**2) / (2 * T**3)
        a4 = (-30 * h + (14 * v1 + 16 * v0) * T +
              (3 * acc0 - 2 * acc1) * T**2) / (2 * T**4)
        a5 = (12 * h - 6 * (v1 + v0) * T + (acc1 - acc0) * T**2) / (2 * T**5)
    else:
        a0 = q0
        a1 = a2 = a3 = a4 = a5 = np.zeros(7)
    return T, a0, a1, a2, a3, a4, a5


jp = [
    math.radians(0),
    math.radians(0),
    math.radians(0),
    math.radians(-90),
    math.radians(0),
    math.radians(90),
    math.radians(0)
]

current_joint_position = [
    math.radians(0),
    math.radians(0),
    math.radians(0),
    math.radians(0),
    math.radians(0),
    math.radians(0),
    math.radians(0)
]

current_tcp_position = [0., 0., 0., 0., 0., 0.]
q_start = q_goal = np.zeros(7)
v_start = v_goal = acc_start = acc_goal = np.zeros(7)
a0 = a1 = a2 = a3 = a4 = a5 = np.zeros(7)
q_d = list()
filename = './data/data.csv'
of = open(filename, 'w')

# header = [
#     'time', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'x', 'y', 'z', 'w', 'i',
#     'j', 'k', 'dx', 'dy', 'dz', 'dwx', 'dwy', 'dwz', 'ddx', 'ddy', 'ddz',
#     'ddwx', 'ddwy', 'ddwz'
# ]
header = [
    'time', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'dq1', 'dq2', 'dq3',
    'dq4', 'dq5', 'dq6', 'dq7', 'x', 'y', 'z', 't1', 't2', 't3', 't4', 't5',
    't6', 't7', 'et1', 'et2', 'et3', 'et4', 'et5', 'et6', 'et7'
]
of.writelines(','.join(header) + ',')
of.write('\n')

ext_force = np.array([0., 0., 0., 0., 0., 0.])


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


def joint_servo_control(robot_state, period):
    # start = time.time()
    global time_count, initial_position, redis_count
    global a0, a1, a2, a3, a4, a5, T, q_start, q_goal, q_d
    global jp, current_joint_position, current_tcp_position, ext_force, current_tcp_matrix, current_tcp_q, ext_tau, measured_tau
    global franka_state

    franka_state = robot_state

    if q_d == []:
        q_d = np.array(robot_state.q_d)

    # jp = redis_obj.read("SET_REFERENCE_JOINT_POSITION")

    time_count = time_count + period.toSec()

    if (np.array(jp) != q_goal).any():
        q_start = np.array(q_d)
        v_start = np.array(robot_state.dq_d)
        acc_start = np.array(robot_state.ddq_d)
        q_goal = np.array(jp)
        v_goal = np.zeros(7)
        acc_goal = np.zeros(7)

        T, a0, a1, a2, a3, a4, a5 = polynomial(q_start, q_goal, v_start,
                                               v_goal, acc_start, acc_goal)
        time_count = period.toSec()
        # print(q_goal)

    q_d = calculateDesiredValues(time_count, a0, a1, a2, a3, a4, a5)
    joint_position_obj = JointPositions(q_d)
    current_joint_position = robot_state.q
    current_tcp_position = matrix2q(robot_state.O_T_EE)
    current_tcp_q = matrix2q(robot_state.O_T_EE)
    current_tcp_matrix = robot_state.O_T_EE
    # print(q_d)
    # of.write(str(time_count) + ',')
    # of.write(','.join(str(q) for q in q_d))
    # of.write('\n')
    # joint_position_obj = JointPositions(robot_state.q_d)
    # end = time.time()
    # # print("timi: ", end - start)
    # # print("time_count: ", time_count)
    # get state variables
    jacobian = np.array(model.zeroJacobian(Frame.kEndEffector,
                                           robot_state)).reshape(7, 6)
    measured_tau = robot_state.tau_J
    ext_tau = robot_state.tau_ext_hat_filtered
    if np.linalg.det(jacobian.transpose() @ jacobian) > 1e-6:
        ext_force = np.linalg.pinv(jacobian) @ ext_tau
    # print(ext_force)
    # print(gravity)
    # of.write(str(time_count) + ',')
    # for x in robot_state.q:
    #     of.write(str(x))
    #     of.write(',')
    # for x in robot_state.dq:
    #     of.write(str(x))
    #     of.write(',')
    # for x in robot_state.O_T_EE[12:15]:
    #     of.write(str(x))
    #     of.write(',')
    # # for x in robot_state.O_dP_EE_c:
    # #     of.write(str(x))
    # #     of.write(',')
    # # for x in robot_state.O_ddP_EE_c:
    # #     of.write(str(x))
    # #     of.write(',')

    # for x in robot_state.tau_J:
    #     of.write(str(x))
    #     of.write(',')
    # for x in robot_state.tau_ext_hat_filtered:
    #     of.write(str(x))
    #     of.write(',')
    # of.write('\n')
    # tau_d = np.zeros(7, dtype=float)
    # desired_force_torque = np.zeros(6, dtype=float)
    # tau_cmd = np.zeros(7, dtype=float)
    # tau_ext = np.zeros(7, dtype=float)

    # desired_force_torque[2] = desired_mass * -9.81
    # tau_ext = tau_measured - gravity - initial_tau_ext
    # tau_d = jacobian @ desired_force_torque
    # tau_error_integral = tau_error_integral + period.toSec() * (tau_d.T -
    #                                                             tau_ext.T)

    # print("jacobian_array: ", jacobian_array)
    # print("jacobian: ", jacobian)
    # print("desired_force_torque: ", desired_force_torque)
    # print("tau_d: ", tau_d)
    # print("tau_error_integral: ", tau_error_integral)
    # FF + PI control
    # tau_cmd = tau_d + k_p * (tau_d.T - tau_ext.T) + k_i * tau_error_integral

    # # Smoothly update the mass to reach the desired target value
    # desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass
    return joint_position_obj


redis_obj = RedisBase("localhost", 6, 1)
thread_obj = Thread(target=rw_redis_data)
thread_obj.start()

time_count = 0.0
initial_position = [0, 0, 0, 0, 0, 0, 0]
q_goal = [
    0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4
]
# motion_generator = MotionGenerator(0.5, q_goal)
redis_count = 0

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")
franka_api_obj.set_default_behavior()
# load the kinematics and dynamics model
model = franka_api_obj.loadModel()
initial_state = franka_api_obj.readOnce()
franka_state = franka_api_obj.readOnce()
initial_tau_ext = np.zeros(7, dtype=float)
tau_error_integral = np.zeros(7, dtype=float)
# Bias torque sensor
gravity_array = model.gravity(initial_state, [0., 0., -9.81])
initial_tau_measured = np.copy(initial_state.tau_J).reshape(7, 1)
initial_gravity = np.copy(gravity_array).reshape(7, 1)
initial_tau_ext = initial_tau_measured - initial_gravity

cjp = initial_state.q_d
redis_obj.write("SET_REFERENCE_JOINT_POSITION", cjp)
lower_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
upper_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
lower_force_thresholds = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
upper_force_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

franka_api_obj.set_collision_behavior(lower_torque_thresholds,
                                      upper_torque_thresholds,
                                      lower_force_thresholds,
                                      upper_force_thresholds)

error_count = 0
while True:
    time_count = 0.0
    if franka_api_obj.robot_control(joint_servo_control,
                                    ControllerMode.kJointImpedance, True,
                                    kDefaultCutoffFrequency) == False:
        error_count = error_count + 1
        franka_api_obj.set_default_behavior()

        jp = np.array([
            math.radians(0),
            math.radians(0),
            math.radians(0),
            math.radians(-90),
            math.radians(0),
            math.radians(90),
            math.radians(0)
        ])

        q_start = q_goal = np.zeros(7)
        v_start = v_goal = acc_start = acc_goal = np.zeros(7)
        a0 = a1 = a2 = a3 = a4 = a5 = np.zeros(7)
        q_d = list()

        time.sleep(2)
        print('auto recover from errors')
