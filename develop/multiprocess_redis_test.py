
import redis
import msgpack
from enum import Enum
import time
import math
import copy
from threading import Thread
import threading
import multiprocessing
from pyfranka.franka_pybind import *


def robot_state_dic(robot_state):
    data_dic = {}
    data_dic["O_T_EE"] = robot_state.O_T_EE
    data_dic["O_T_EE_d"] = robot_state.O_T_EE_d
    data_dic["F_T_EE"] = robot_state.F_T_EE
    data_dic["F_T_NE"] = robot_state.F_T_NE
    data_dic["NE_T_EE"] = robot_state.NE_T_EE
    data_dic["EE_T_K"] = robot_state.EE_T_K
    data_dic["m_ee"] = robot_state.m_ee
    data_dic["I_ee"] = robot_state.I_ee
    data_dic["F_x_Cee"] = robot_state.F_x_Cee
    data_dic["m_load"] = robot_state.m_load

    data_dic["I_load"] = robot_state.I_load
    data_dic["F_x_Cload"] = robot_state.F_x_Cload
    data_dic["m_total"] = robot_state.m_total
    data_dic["I_total"] = robot_state.I_total
    data_dic["F_x_Ctotal"] = robot_state.F_x_Ctotal
    data_dic["elbow"] = robot_state.elbow
    data_dic["elbow_d"] = robot_state.elbow_d
    data_dic["elbow_c"] = robot_state.elbow_c

    data_dic["delbow_c"] = robot_state.delbow_c
    data_dic["ddelbow_c"] = robot_state.ddelbow_c
    data_dic["tau_J"] = robot_state.tau_J
    data_dic["tau_J_d"] = robot_state.tau_J_d
    data_dic["dtau_J"] = robot_state.dtau_J
    data_dic["q"] = robot_state.q
    data_dic["q_d"] = robot_state.q_d
    data_dic["dq"] = robot_state.dq
    data_dic["dq_d"] = robot_state.dq_d
    data_dic["ddq_d"] = robot_state.ddq_d

    data_dic["joint_contact"] = robot_state.joint_contact
    data_dic["cartesian_contact"] = robot_state.cartesian_contact
    data_dic["joint_collision"] = robot_state.joint_collision
    data_dic["cartesian_collision"] = robot_state.cartesian_collision
    data_dic["tau_ext_hat_filtered"] = robot_state.tau_ext_hat_filtered
    data_dic["O_F_ext_hat_K"] = robot_state.O_F_ext_hat_K
    data_dic["K_F_ext_hat_K"] = robot_state.K_F_ext_hat_K
    data_dic["O_dP_EE_d"] = robot_state.O_dP_EE_d
    data_dic["O_T_EE_c"] = robot_state.O_T_EE_c
    data_dic["O_dP_EE_c"] = robot_state.O_dP_EE_c
    data_dic["O_ddP_EE_c"] = robot_state.O_ddP_EE_c
    data_dic["theta"] = robot_state.theta
    data_dic["dtheta"] = robot_state.dtheta
    # data_dic["current_errors"] = robot_state.current_errors
    # data_dic["last_motion_errors"] = robot_state.last_motion_errors
    data_dic["control_command_success_rate"] = robot_state.control_command_success_rate
    # data_dic["robot_mode"] = robot_state.robot_mode
    # data_dic["time"] = robot_state.time
    return data_dic



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

redis_thread_obj = RedisBase("localhost", 1, 1)

data_share_dic = multiprocessing.Manager().dict()

def rw_redis_data(data_share_dic):
    
    while True:
        
        t1 = time.time()

        if "robot_state" in data_share_dic.keys():
            # redis_thread_obj.write("robot_state", data_share_dic["robot_state"])
            for key, value in data_share_dic["robot_state"].items():
                redis_thread_obj.write(key, value) 

        t2 = time.time()
        print("redis thead", t2-t1)
        

# process_obj = multiprocessing.Process(target=rw_redis_data, args=(data_share_dic, ))
# process_obj.start()

### joint_position_control ###
joint_time_count = 0.0
joint_initial_position = [0, 0, 0, 0, 0, 0, 0]

def joint_position_control(robot_state, period):
    global joint_time_count, joint_initial_position, franka_state

    data_share_dic["robot_state"] = robot_state_dic(robot_state)

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

    return joint_position_obj

### cartesian_position_control ###
cartesian_time_count = 0.0
cartesian_initial_position = [0] * 16
kRadius = 0.3
def cartesian_position_control(robot_state, period):

    global cartesian_time_count, cartesian_initial_position, franka_state

    data_share_dic["robot_state"] = robot_state_dic(robot_state)

    if cartesian_time_count == 0.0:
        cartesian_initial_position = robot_state.O_T_EE_c

    cartesian_time_count = cartesian_time_count + period.toSec()

    angle = math.pi / 4 * (1 - math.cos(math.pi / 5 * cartesian_time_count))
    delta_x = kRadius * math.sin(angle)
    delta_z = kRadius * (math.cos(angle) - 1)

    new_pose = copy.copy(cartesian_initial_position)
    new_pose[12] = new_pose[12] + delta_x
    new_pose[14] = new_pose[14] + delta_z

    cartesian_pose_obj = CartesianPose(new_pose)

    if cartesian_time_count >= 5:
        return CartesianPoseMotionFinished(cartesian_pose_obj)

    return cartesian_pose_obj 


### init ###
franka_gripper_api_obj = Gripper("192.170.10.100")

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")

def joint_pose_control_mode():
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

def cartesian_pose_control_mode():
    global cartesian_time_count, cartesian_initial_position
    cartesian_time_count = 0.0
    cartesian_initial_position = [0] * 16

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

    re = franka_api_obj.robot_control(cartesian_pose_handle=cartesian_position_control)
    return re

def grasp_control_mode():
    grasping_width = 0.079  # m

    homing = True

    if homing:
        franka_gripper_api_obj.homing()

    # Check for the maximum grasping width.
    gripper_state = franka_gripper_api_obj.readOnce()

    if gripper_state.max_width < grasping_width:
        print("Object is too large for the current fingers on the gripper.")
        return False

    # Grasp the object.
    if not franka_gripper_api_obj.grasp(grasping_width, 0.1, 60, 0.005, 0.005):
        print("Failed to grasp object.")
        return False

    print("Grasped object, will release it now.")
    franka_gripper_api_obj.stop()
    return True
                    

class ControlState(Enum):

    GRASP_STATE = 1  
    POSITION_CONTROL_STATE = 2 
    CARTESIAN_CONTROL_STATE = 3 

    ERROR_STATE = 4

def run_state_machine():

    control_state = ControlState.GRASP_STATE
    redis_obj = RedisBase("localhost", 1, 1)

    while True:
        # TASK_FINISHED = redis_obj.read("TASK_FINISHED")
        # if TASK_FINISHED == True:
        #     return

        if control_state == ControlState.GRASP_STATE:
            control_state = ControlState.POSITION_CONTROL_STATE
            
        elif control_state == ControlState.POSITION_CONTROL_STATE:
            result = joint_pose_control_mode()
            if result == True:
                control_state = ControlState.CARTESIAN_CONTROL_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_STATE

        elif control_state == ControlState.CARTESIAN_CONTROL_STATE:
            result = cartesian_pose_control_mode()
            if result == True:
                control_state = ControlState.GRASP_STATE
            else:
                control_state = ControlState.CARTESIAN_CONTROL_STATE

if __name__ == "__main__":
    # joint_pose_control_mode()
    # cartesian_pose_control_mode()
    # grasp_control_mode()

    process_obj = multiprocessing.Process(target=rw_redis_data, args=(data_share_dic, ))
    process_obj.start()

    run_state_machine()


