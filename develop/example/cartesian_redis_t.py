from pyfranka.franka_pybind import *
import sys
import math
import time
import copy

import redis
import msgpack

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


time_count = 0.0
initial_position = [0, 0, 0, 0, 0, 0, 0]

q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
motion_generator = MotionGenerator(0.5, q_goal)
kRadius = 0.3

redis_obj = RedisBase("localhost", 1, 1)

def cartesian_control_test(robot_state, period):

    start = time.time()

    global time_count, initial_position

    if time_count == 0.0:
        initial_position = robot_state.O_T_EE_c


    time_count = time_count + period.toSec()


    angle = math.pi / 4 * (1 - math.cos(math.pi / 5 * time_count))
    delta_x = kRadius * math.sin(angle)
    delta_z = kRadius * (math.cos(angle) - 1)

    new_pose = copy.copy(initial_position)
    new_pose[12] = new_pose[12] + delta_x
    new_pose[14] = new_pose[14] + delta_z

    # print(robot_state.O_T_EE_c)

    cartesian_pose_obj = CartesianPose(new_pose)

    redis_obj.write("O_T_EE", robot_state.O_T_EE)
    # test = redis_obj.read("test")

    if time_count >= 5:
        time_count = 0
        return CartesianPoseMotionFinished(cartesian_pose_obj)

    end = time.time()

    print(end-start)

    return cartesian_pose_obj


franka_api_obj = FrankaApi()

franka_api_obj.init_config("192.170.10.100")

# franka_api_obj.request_robot_state()
franka_api_obj.set_default_behavior()

franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)

franka_api_obj.set_collision_behavior()

franka_api_obj.robot_control(cartesian_pose_handle=cartesian_control_test)







