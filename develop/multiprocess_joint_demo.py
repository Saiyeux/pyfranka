
from redis_base import RedisBase
from multiprocessing import Process
import time
from data_share import data_share_obj
from pyfranka.franka_pybind import *
from math_cal import matrix2q
import math


def set_franka_status(robot_state):
    data_share_obj.TIMESTAMP.value = float(time.time())
    data_share_obj.O_T_EE[:] = matrix2q(robot_state.O_T_EE)
    data_share_obj.tau_J[:] = robot_state.tau_J
    data_share_obj.q[:] = robot_state.q
    data_share_obj.dq[:] = robot_state.dq
    data_share_obj.tau_ext_hat_filtered[:] = robot_state.tau_ext_hat_filtered
    data_share_obj.O_F_ext_hat_K[:] = robot_state.O_F_ext_hat_K

def get_franka_status():
    TIMESTAMP = data_share_obj.TIMESTAMP.value
    O_T_EE = data_share_obj.O_T_EE[:]
    tau_J = data_share_obj.tau_J[:]
    q = data_share_obj.q[:]
    dq = data_share_obj.dq[:]
    tau_ext_hat_filtered = data_share_obj.tau_ext_hat_filtered[:]
    O_F_ext_hat_K = data_share_obj.O_F_ext_hat_K[:]

### joint_position_control ###
joint_time_count = 0.0
joint_initial_position = [0, 0, 0, 0, 0, 0, 0]
def joint_position_control(robot_state, period):
    t1 = time.time()
    global joint_time_count, joint_initial_position
    set_franka_status(robot_state)
    get_franka_status()
    if joint_time_count == 0.0:
        joint_initial_position = robot_state.q_d
    joint_time_count = joint_time_count + period.toSec()
    delta_angle = math.pi / 8.0 * (1 - math.cos(math.pi / 2.5 * joint_time_count))

    position = [joint_initial_position[0], joint_initial_position[1],
                joint_initial_position[2], joint_initial_position[3],
                joint_initial_position[4], joint_initial_position[5]+ delta_angle,
                joint_initial_position[6] ]

    joint_position_obj = JointPositions(position)
    if joint_time_count >= 5:
        return JointPositionsMotionFinished(joint_position_obj)

    # joint_position_obj = JointPositions(robot_state.q_d)

    t2 = time.time()
    if (t2 - t1) >= 0.0003:
        print("joint_pose_control_mode: ", t2 - t1)
    
    return joint_position_obj

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")


def joint_pose_control_mode():
    # t1 = time.time()
    global joint_time_count, joint_initial_position
    joint_time_count = 0.0
    joint_initial_position = [0, 0, 0, 0, 0, 0, 0]

    q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
    motion_generator = MotionGenerator(0.5, q_goal)

    franka_api_obj.set_default_behavior()
    re = franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)
    if re == False:
        return re

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

    re = franka_api_obj.robot_control(joint_positions_handle=joint_position_control)

    return re

def joint_pose_control_mode_loop():
    count = 0
    while True:
        try:
            result = joint_pose_control_mode()
            if result == False:
                count+=1
                print("joint_pose_control_mode error: ", count)
            data_share_obj.error_count.value = count
        except Exception as e:
            print(Exception, e)


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

        redis_obj.write("error_count", data_obj.error_count.value)

        t2 = time.time()
        if (t2 - t1) < 0.001:
            time.sleep(0.001 - float(t2 - t1))
        # print("redis_loop_process: ", t2 - t1)

def redis_loop_process2(data_obj):
    redis_obj = RedisBase("localhost", 1, 1)
    while True:
        t1 = time.time()

        TIMESTAMP = redis_obj.read("TIMESTAMP")
        O_T_EE = redis_obj.read("O_T_EE")
        tau_J = redis_obj.read("tau_J")
        tau_J = redis_obj.read("tau_J")
        q = redis_obj.read("q")
        dq = redis_obj.read("dq")
        tau_ext_hat_filtered = redis_obj.read("tau_ext_hat_filtered")
        O_F_ext_hat_K = redis_obj.read("O_F_ext_hat_K")

        error_count = redis_obj.read("error_count")

        t2 = time.time()
        if (t2 - t1) < 0.001:
            time.sleep(0.001 - float(t2 - t1))
        # print("redis_loop_process2: ", t2 - t1)


redis_thread_obj = Process(target=redis_loop_process, args=(data_share_obj,))
redis_thread_obj2 = Process(target=redis_loop_process2, args=(data_share_obj,))
process_obj = Process(target=joint_pose_control_mode_loop)
    

print("start server..")
redis_thread_obj.start()
redis_thread_obj2.start()
process_obj.start()

# while True:
#     re = franka_api_obj.robot_control(joint_positions_handle=joint_position_control)

redis_thread_obj.join()
redis_thread_obj2.join()
process_obj.join()
