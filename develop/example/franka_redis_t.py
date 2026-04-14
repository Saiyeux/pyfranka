from pyfranka.franka_pybind import *
import sys
import math
import time

import redis
import msgpack
from threading import Thread

class RedisBase():
    def __init__(self, host, database, socket_connect_timeout):
        self.__r = redis.StrictRedis(host=host, port=6379, db=database, socket_connect_timeout=socket_connect_timeout)

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

redis_obj = RedisBase("localhost", 1, 1)

O_T_EE = []
def rw_redis_data():
    global O_T_EE
    while True:
        # t1 = time.time()
        redis_obj.write("O_T_EE", O_T_EE)
        # t2 = time.time()
        # print("redis thead", t2-t1)

# thread_obj = Thread(target=rw_redis_data)
# thread_obj.start()



time_count = 0.0
initial_position = [0, 0, 0, 0, 0, 0, 0]

q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
motion_generator = MotionGenerator(0.5, q_goal)

redis_count = 0
total_time = 0
error_count = 0

def position_control_test(robot_state, period):

    start = time.time()

    global time_count, initial_position, redis_count, O_T_EE, total_time

    if time_count == 0.0:
        initial_position = robot_state.q_d

    time_count = time_count + period.toSec()

    delta_angle = math.pi / 8.0 * (1 - math.cos(math.pi / 2.5 * time_count))

    # position = [initial_position[0], initial_position[1],
    #             initial_position[2], initial_position[3] + delta_angle,
    #             initial_position[4] + delta_angle, initial_position[5],
    #             initial_position[6] + delta_angle]

    position = [initial_position[0], initial_position[1],
                initial_position[2], initial_position[3],
                initial_position[4], initial_position[5]+ delta_angle,
                initial_position[6] ]


    joint_position_obj = JointPositions(position)


    redis_count = redis_count + 1
    total_time = total_time + 1
    # if redis_count > 20:
    #     redis_obj.write("O_T_EE", robot_state.O_T_EE)
    #     redis_count = 0
    redis_obj.write("O_T_EE", robot_state.O_T_EE)
    O_T_EE = redis_obj.read("O_T_EE")

    if time_count >= 5:
        # joint_position_obj.motion_finished = True
        time_count = 0.0
        return JointPositionsMotionFinished(joint_position_obj)

    end = time.time()
    print("timi: ", end-start)
    print("total_time: ", total_time / 1000.0)
    print("error_count: ", error_count)

    return joint_position_obj


franka_api_obj = FrankaApi()

franka_api_obj.init_config("192.170.10.100")

# franka_api_obj.request_robot_state()

franka_api_obj.set_default_behavior()

re = franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)

if re == False:
    error_count = error_count + 1

lower_torque_thresholds_acceleration = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
upper_torque_thresholds_acceleration = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
lower_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
upper_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

franka_api_obj.set_collision_behavior(lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                                      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                                      lower_force_thresholds_nominal, upper_force_thresholds_nominal)

while True:

    time_count = 0.0

    re = franka_api_obj.robot_control(joint_positions_handle=position_control_test)

    if re == False:
        error_count = error_count + 1

        franka_api_obj.set_default_behavior()
        r = franka_api_obj.robot_control(joint_positions_handle=position_control_test)
        franka_api_obj.set_collision_behavior(lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                                              lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                              lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                                              lower_force_thresholds_nominal, upper_force_thresholds_nominal)
        if r == False:
            error_count = error_count + 1
        time.sleep(2)




