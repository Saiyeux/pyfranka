from pyfranka.franka_pybind import *
import math
import time
import numpy as np
import redis
import msgpack
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
    global jp, current_joint_position, current_tcp_position
    while True:
        t1 = time.time()
        jp = redis_obj.read("SET_REFERENCE_JOINT_POSITION")
        redis_obj.write("GET_REALTIME_JOINT_POSITION", current_joint_position)
        redis_obj.write("GET_REALTIME_TCP_POSITION", current_tcp_position)
        t2 = time.time()
        print("redis thead", t2 - t1)


def calculateDesiredValues(t, a0, a1, a2, a3, a4, a5):
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
    T = max(T, 20e-3)

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
of = open('data.csv', 'w')


def joint_servo_control(robot_state, period):
    # start = time.time()
    global time_count, initial_position, redis_count
    global a0, a1, a2, a3, a4, a5, T, q_start, q_goal, q_d
    global jp, current_joint_position, current_tcp_position

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
    current_tcp_position = robot_state.O_T_EE
    # print(q_d)
    # of.write(str(time_count) + ',')
    # of.write(','.join(str(q) for q in q_d))
    # of.write('\n')
    # joint_position_obj = JointPositions(robot_state.q_d)
    # end = time.time()
    # # print("timi: ", end - start)
    # # print("time_count: ", time_count)
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
# franka_api_obj.request_robot_state()
franka_api_obj.set_default_behavior()
# franka_api_obj.robot_control(motion_generator.operator,
#                              ControllerMode.kJointImpedance, True,
#                              kDefaultCutoffFrequency)
franka_api_obj.set_collision_behavior()

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
