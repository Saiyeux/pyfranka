
import numpy as np
import math
import time
from pyfranka.franka_pybind import FrankaApi, RobotMode, MotionGenerator, CartesianVelocities, CartesianVelocitiesFinished


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


# Set additional parameters always before the control loop, NEVER in the control loop!
# Set the joint impedance.
franka.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000])


"""
这个demo的功能是，franka的cartesian velocity control，具体可以运行看效果，目前暂时设定的时间为运动５秒后就停止运动。

停止运动的代码接口为　CartesianVelocitiesFinished，具体用法见下面代码demo。

函数cartesian_position_control本质上是一个回调函数，这里函数名可以不一样，但是参数名要一样。例如改成函数control(robot_state, period)也是可以的。

简单的用法为，先定义回调函数，如何将回调函数赋给robot_control即可运行 

def control(robot_state, period):
    
    output = ...

    cartesian_velocities_obj = CartesianVelocities(output)

    if 要停止机械臂运动:
        return CartesianVelocitiesFinished(cartesian_velocities_obj)

    return cartesian_velocities_obj
    
franka.robot_control(cartesian_velocities_handle=control)

"""
time_max = 4.0
v_max = 0.05
angle = math.pi / 4.0
time_count = 0.0
def cartesian_velocity_control(robot_state, period):
    global time_max, v_max, angle, time_count

    time_count += period.toSec()

    cycle = math.floor(math.pow(-1.0, (time_count - math.fmod(time_count, time_max)) / time_max))
    v = cycle * v_max / 2.0 * (1.0 - math.cos(2.0 * math.pi / time_max * time_count))
    v_x = math.cos(angle) * v
    v_z = -math.sin(angle) * v

    output = [v_x, 0.0, v_z, 0.0, 0.0, 0.0]

    cartesian_velocities_obj = CartesianVelocities(output)

    if time_count >= time_max * 2:
        return CartesianVelocitiesFinished(cartesian_velocities_obj)

    return cartesian_velocities_obj

franka.robot_control(cartesian_velocities_handle=cartesian_velocity_control)

