import sys
import numpy as np
import time
import math
from pyfranka.franka_pybind import FrankaApi, RobotMode, MotionGenerator, JointPositions, JointPositionsMotionFinished

# 对Franka的一些参数进行初始化, 基本上都可以这样进行初始化
FRANKA_IPADDR = '192.170.10.100'

franka = FrankaApi()
franka.init_config(FRANKA_IPADDR, log_size=5000)
franka.set_default_behavior()

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

franka.set_collision_behavior(
    lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
    lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
    lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
    lower_force_thresholds_nominal, upper_force_thresholds_nominal)


# 这里的代码就是检查一下franka当前的状态，是不是已经ready了
Ready = False
for i in range(3):
    franka_state = franka.readOnce()
    if (franka_state.robot_mode == RobotMode.kIdle):
        Ready = True
    else:
        print("Not Ready")
        if franka_state.robot_mode in [RobotMode.kReflex, ]:
            franka.automatic_error_recovery()

print("Ready")


"""
    这三行代码可注释，这里写，只是用作参考以及测试
    
    这里的MotionGenerator的作用就是将franka机械臂移动至一个默认的位置，
    这个默认的位置暂时是根据libfranka给的默认位置，后面也可以自己设定。
"""
q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
motion_generator = MotionGenerator(0.5, q_goal)
franka.robot_control(joint_positions_handle=motion_generator.operator)


"""
这个demo的功能是，franka绕一个轴反复移动一个角度，具体可以运行看效果，目前暂时设定的时间为运动５秒后就停止运动。

停止运动的代码接口为　JointPositionsMotionFinished，具体用法见下面代码demo。

函数joint_position_control本质上是一个回调函数，这里函数名可以不一样，但是参数名要一样。例如改成函数control(robot_state, period)也是可以的。

简单的用法为，先定义回调函数，如何将回调函数赋给robot_control即可运行 

def control(robot_state, period):
    
    # position的数据格式和robot_state.q_d相同，position即目标位置
    position = ...
    
    joint_position_obj = JointPositions(position)
    
    if 要停止机械臂运动:
        return JointPositionsMotionFinished(joint_position_obj)

    return joint_position_obj
    
franka.robot_control(joint_positions_handle=control)

"""
time_count = 0.0
initial_position = [0, 0, 0, 0, 0, 0, 0]
def joint_position_control(robot_state, period):

    start = time.time()

    global time_count, initial_position

    if time_count == 0.0:
        initial_position = robot_state.q_d

    time_count = time_count + period.toSec()

    delta_angle = math.pi / 8.0 * (1 - math.cos(math.pi / 2.5 * time_count))

    position = [initial_position[0], initial_position[1],
                initial_position[2], initial_position[3] + delta_angle,
                initial_position[4] + delta_angle, initial_position[5],
                initial_position[6] + delta_angle]

    joint_position_obj = JointPositions(position)

    if time_count >= 5:
        return JointPositionsMotionFinished(joint_position_obj)

    end = time.time()
    print(end-start)

    return joint_position_obj

franka.robot_control(joint_positions_handle=joint_position_control)

