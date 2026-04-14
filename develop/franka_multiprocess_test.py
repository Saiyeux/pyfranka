
from redis_base import RedisBase
from multiprocessing import Process
import time
from data_share import data_share_obj
from pyfranka.franka_pybind import *
from math_cal import matrix2q


def set_franka_status(robot_state):
    data_share_obj.TIMESTAMP.value = float(time.time())
    data_share_obj.O_T_EE[:] = matrix2q(robot_state.O_T_EE)
    data_share_obj.tau_J[:] = robot_state.tau_J
    data_share_obj.q[:] = robot_state.q
    data_share_obj.dq[:] = robot_state.dq
    data_share_obj.tau_ext_hat_filtered[:] = robot_state.tau_ext_hat_filtered
    data_share_obj.O_F_ext_hat_K[:] = robot_state.O_F_ext_hat_K



### joint_position_control ###
# joint_time_count = 0.0
# joint_initial_position = [0, 0, 0, 0, 0, 0, 0]
def joint_position_control(robot_state, period):
    t1 = time.time()
    set_franka_status(robot_state)

    joint_position_obj = JointPositions(robot_state.q_d)

    t2 = time.time()
    # print(t2 - t1)
    return joint_position_obj

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")


initial_state = set_franka_status(franka_api_obj.readOnce())


def redis_loop_process(data_obj):
    redis_obj = RedisBase("localhost", 1, 1)
    while True:
        t1 = time.time()

        redis_obj.write("TIMESTAMP", data_obj.TIMESTAMP.value)
        redis_obj.write("O_T_EE", data_obj.O_T_EE[:])
        redis_obj.write("tau_J", data_obj.tau_J[:])
        redis_obj.write("tau_J", data_obj.tau_J[:])
        redis_obj.write("q", data_obj.q[:])
        redis_obj.write("dq", data_obj.dq[:])
        redis_obj.write("tau_ext_hat_filtered", data_obj.tau_ext_hat_filtered[:])
        redis_obj.write("O_F_ext_hat_K", data_obj.O_F_ext_hat_K[:])

        t2 = time.time()
        if (t2 - t1) < 0.001:
            time.sleep(0.001 - float(t2 - t1))
        print(t2 - t1)




redis_thread_obj = Process(target=redis_loop_process, args=(data_share_obj,))

print("start server..")
redis_thread_obj.start()

while True:
    re = franka_api_obj.robot_control(joint_positions_handle=joint_position_control)

redis_thread_obj.join()
