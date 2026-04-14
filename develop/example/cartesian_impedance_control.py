
from pyfranka.franka_pybind import *
import numpy as np
import math
from math_cal import *

# Compliance parameters
translational_stiffness = 150.0
rotational_stiffness = 10.0

stiffness = np.zeros((6, 6), dtype = float)
damping = np.zeros((6, 6), dtype = float)

stiffness[0:3, 0:3] = translational_stiffness * np.eye(3)
stiffness[3:6, 3:6] = rotational_stiffness * np.eye(3)

damping[0:3, 0:3] = 2 * math.sqrt(translational_stiffness) * np.eye(3)
damping[3:6, 3:6] = 2 * math.sqrt(rotational_stiffness) * np.eye(3)

# connect to robot
franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")

franka_api_obj.automatic_error_recovery()

franka_api_obj.set_default_behavior()
# load the kinematics and dynamics model
model = franka_api_obj.loadModel()

initial_state = franka_api_obj.readOnce()

# equilibrium point is the initial position
initial_transform_mat = np.copy(initial_state.O_T_EE).reshape(4,4).T
position_d = initial_transform_mat[0:3, 3].T  # [0:3,0:3]
orientation_d = M2O(initial_transform_mat)

# set collision behavior
lower_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
upper_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
lower_force_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
upper_force_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
franka_api_obj.set_collision_behavior(lower_torque_thresholds, upper_torque_thresholds,
                                      lower_force_thresholds, upper_force_thresholds)

# define callback for the torque control loop
def torques_controller(robot_state, period):
    global initial_transform_mat, position_d, orientation_d, stiffness, damping

    # get state variables
    coriolis_array = model.coriolis(robot_state)
    jacobian_array = model.zeroJacobian(Frame.kEndEffector, robot_state)

    # convert to Eigen
    coriolis = np.copy(coriolis_array).reshape(7, 1)
    jacobian = np.copy(jacobian_array).reshape(7, 6)
    q = np.copy(robot_state.q).reshape(7, 1)
    dq = np.copy(robot_state.dq).reshape(7, 1)
    transform_mat = np.copy(robot_state.O_T_EE).reshape(4,4).T
    position = transform_mat[0:3, 3].T
    orientation = M2O(transform_mat)

    # compute error to desired equilibrium pose
    # position error
    error = np.zeros(6, dtype=float)
    error[0:3] = position - position_d

    # orientation error
    # "difference" quaternion
    if np.dot(orientation_d, orientation) < 0.0:
        orientation = orientation * (-1)

    # "difference" quaternion
    error_quaternion = np.mat(transform_mat[0:3, 0:3]).I @ initial_transform_mat[0:3, 0:3]
    error[3:6] = M2O(error_quaternion)[0:3]

    # Transform to base frame
    error[3:6] = transform_mat[0:3, 0:3] * (-1) @ error[3:6]

    # compute control
    tau_task = np.zeros(7, dtype=float)
    tau_d = np.zeros(7, dtype=float)

    # Spring damper system with damping ratio=1
    tau_task = jacobian @ (((-1) * stiffness @ error).reshape(6, 1) - damping @ (jacobian.T @ dq))
    tau_d = tau_task + coriolis

    tau_d_array = tau_d.reshape(1, 7)[0].tolist()
    # print("tau_d_array: \n", tau_d_array)

    torques_obj = Torques(tau_d_array)

    return torques_obj

# while True:
franka_api_obj.robot_control(torques_handle = torques_controller)








