
import redis
import msgpack
from enum import Enum
import time
import math
import copy
from pyfranka.franka_pybind import *


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

### joint_position_control 1 ###
joint_time_count = 0.0
joint_initial_position = [0, 0, 0, 0, 0, 0, 0]

def joint_position_control_1(robot_state, period):
    global joint_time_count, joint_initial_position
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

### joint_position_control 2###
def joint_position_control_2(robot_state, period):
    global joint_time_count, joint_initial_position
    if joint_time_count == 0.0:
        joint_initial_position = robot_state.q_d
    joint_time_count = joint_time_count + period.toSec()
    delta_angle = math.pi / 8.0 * (1 - math.cos(math.pi / 2.5 * joint_time_count))

    position = [joint_initial_position[0], joint_initial_position[1],
                joint_initial_position[2], joint_initial_position[3] + delta_angle,
                joint_initial_position[4] + delta_angle, joint_initial_position[5],
                joint_initial_position[6] ]

    joint_position_obj = JointPositions(position)
    if joint_time_count >= 5:
        return JointPositionsMotionFinished(joint_position_obj)

    return joint_position_obj


### init ###
franka_gripper_api_obj = Gripper("192.170.10.100")

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")


def joint_pose_control_mode_1():
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

    re = franka_api_obj.robot_control(joint_positions_handle=joint_position_control_1)
    
    return re

def joint_pose_control_mode_2():
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

    re = franka_api_obj.robot_control(joint_positions_handle=joint_position_control_2)
    
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
    POSITION_CONTROL_1_STATE = 2 
    POSITION_CONTROL_2_STATE = 3 

    ERROR_STATE = 4

def run_state_machine():

    control_state = ControlState.GRASP_STATE
    redis_obj = RedisBase("localhost", 1, 1)

    while True:
        # TASK_FINISHED = redis_obj.read("TASK_FINISHED")
        # if TASK_FINISHED == True:
        #     return

        if control_state == ControlState.GRASP_STATE:
            result = grasp_control_mode()
            if result == True:
                control_state = ControlState.POSITION_CONTROL_1_STATE
            else:
                control_state = ControlState.GRASP_STATE
            
        elif control_state == ControlState.POSITION_CONTROL_1_STATE:
            result = joint_pose_control_mode_1()
            if result == True:
                control_state = ControlState.POSITION_CONTROL_2_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_1_STATE

        elif control_state == ControlState.POSITION_CONTROL_2_STATE:
            result = joint_pose_control_mode_2()
            if result == True:
                control_state = ControlState.GRASP_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_2_STATE

if __name__ == "__main__":
    # joint_pose_control_mode_1()
    # joint_pose_control_mode_2()
    # grasp_control_mode()

    run_state_machine()
