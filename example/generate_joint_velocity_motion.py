import sys
import numpy as np
import time
import math
from pyfranka.franka_pybind import FrankaApi, RobotMode, MotionGenerator, JointVelocities, JointVelocitiesFinished

# 对Franka的一些参数进行初始化, 基本上都可以这样进行初始化
FRANKA_IPADDR = '192.170.10.100'

franka = FrankaApi()
franka.init_config(FRANKA_IPADDR, log_size=5000)
franka.set_default_behavior()

# Set the collision behavior.
lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
lower_torque_thresholds_acceleration = [
    20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0
]
upper_torque_thresholds_acceleration = [
    20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0
]
lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
lower_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
upper_force_thresholds_acceleration = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

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

franka.set_collision_behavior(
    lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
    lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
    lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
    lower_force_thresholds_nominal, upper_force_thresholds_nominal)


time_max = 1.0
omega_max = 1.0
time_count = 0.0
def joint_velocities_control(robot_state, period):
    global time_count

    time_count = time_count + period.toSec()

    cycle = np.floor(math.pow(-1.0, (time_count - math.fmod(time_count, time_max)) / time_max))
    omega = cycle * omega_max / 2.0 * (1.0 - math.cos(2.0 * math.pi / time_max * time_count))
    
    r = np.array([omega/2, 0.0, 0.0, omega/2, omega/2, omega/2, omega/2])
    print(r)
    joint_velocities_obj = JointVelocities(r)

    if time_count > (2 * time_max):
        return JointVelocitiesFinished(joint_velocities_obj)

    return joint_velocities_obj


franka.robot_control(joint_velocities_handle=joint_velocities_control)

