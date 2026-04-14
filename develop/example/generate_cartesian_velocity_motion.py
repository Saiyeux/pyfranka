
from pyfranka.franka_pybind import *
import numpy as np
import math

franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")
franka_api_obj.set_default_behavior()

# First move the robot to a suitable joint configuration
q_goal = [0, math.pi / 4 * (-1), 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
motion_generator = MotionGenerator(0.5, q_goal)
franka_api_obj.robot_control(joint_positions_handle=motion_generator.operator)
print("Finished moving to initial joint configuration.")

# Set additional parameters always before the control loop, NEVER in the control loop!
# Set the joint impedance.
franka_api_obj.set_joint_impedance([3000, 3000, 3000, 2500, 2500, 2000, 2000])

# Set the collision behavior.
lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.]
upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
lower_torque_thresholds_acceleration = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
upper_torque_thresholds_acceleration = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]

lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

franka_api_obj.set_collision_behavior(lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                                      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                                      lower_force_thresholds_nominal, upper_force_thresholds_nominal)

time_max = 4.0
v_max = 0.1
angle = math.pi / 4.0
time = 0.0

def cartesian_velocity_control(robot_state, period):
    global time_max, v_max, angle, time

    time += period.toSec()

    cycle = math.floor(math.pow(-1.0, (time - math.fmod(time, time_max)) / time_max))
    v = cycle * v_max / 2.0 * (1.0 - math.cos(2.0 * math.pi / time_max * time))
    v_x = math.cos(angle) * v
    v_z = -math.sin(angle) * v

    output = [v_x, 0.0, v_z, 0.0, 0.0, 0.0]

    cartesian_velocities_obj = CartesianVelocities(output)

    if time >= time_max * 2:
        return CartesianVelocitiesFinished(cartesian_velocities_obj)

    return cartesian_velocities_obj

franka_api_obj.robot_control(cartesian_velocities_handle=cartesian_velocity_control)
