from pyfranka.franka_pybind import *
import sys
import math
import time


time_count = 0.0
initial_position = [0, 0, 0, 0, 0, 0, 0]

q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
motion_generator = MotionGenerator(0.5, q_goal)

def position_control_test(robot_state, period):

    start = time.time()

    global time_count, initial_position

    if time_count == 0.0:
        initial_position = robot_state.q_d

    initial_position2 = robot_state.q_d

    time_count = time_count + period.toSec()

    delta_angle = math.pi / 8.0 * (1 - math.cos(math.pi / 2.5 * time_count))

    position = [initial_position[0], initial_position[1],
                initial_position[2], initial_position[3] + delta_angle,
                initial_position[4] + delta_angle, initial_position[5],
                initial_position[6] + delta_angle]
    # position = [initial_position[0], initial_position[1],
    #                     initial_position[2], initial_position[3],
    #                     initial_position[4], initial_position[5]+ delta_angle,
    #                     initial_position[6] ]

    joint_position_obj = JointPositions(position)
    # print("delta_angle: ", delta_angle)

    # print("initial_position1: ", initial_position)
    # print("initial_position2: ", initial_position2)
    # print("time_count: ", time_count)
    # print("period:", period.toSec())
    # print("O_T_EE: ", robot_state.O_T_EE)


    if time_count >= 5:
        # joint_position_obj.motion_finished = True
        return JointPositionsMotionFinished(joint_position_obj)

    end = time.time()
    print(end-start)

    return joint_position_obj


franka_api_obj = FrankaApi()

franka_api_obj.init_config("192.170.10.100")

# franka_api_obj.request_robot_state()
franka_api_obj.set_default_behavior()

franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)

franka_api_obj.set_collision_behavior()

franka_api_obj.robot_control(joint_positions_handle=position_control_test)



    


