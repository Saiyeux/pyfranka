from pyfranka.franka_pybind import *
import numpy as np


# parameters
desired_mass = 0
target_mass = 2.0
k_p = 1.0
k_i = 2.0
filter_gain = 0.001

# connect to robot
franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")

franka_api_obj.automatic_error_recovery()

franka_api_obj.set_default_behavior()
# load the kinematics and dynamics model
model = franka_api_obj.loadModel()

# set collision behavior
lower_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
upper_torque_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
lower_force_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
upper_force_thresholds = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
franka_api_obj.set_collision_behavior(lower_torque_thresholds, upper_torque_thresholds,
                                      lower_force_thresholds, upper_force_thresholds)

initial_state = franka_api_obj.readOnce()

initial_tau_ext = np.zeros(7,dtype=float)
tau_error_integral = np.zeros(7,dtype=float)

# Bias torque sensor
gravity_array = model.gravity(initial_state, [0., 0., -9.81])
initial_tau_measured = np.copy(initial_state.tau_J).reshape(7, 1)
initial_gravity = np.copy(gravity_array).reshape(7, 1)
initial_tau_ext = initial_tau_measured - initial_gravity

# init integrator
tau_error_integral = np.zeros(7,dtype=float)

initial_position = np.array([])
time = 0.0

# define callback for the torque control loop
def torques_controller(robot_state, period):
    global time, initial_position, desired_mass, tau_error_integral, k_p, k_i

    if time == 0.0:
        initial_position = np.array([robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                    robot_state.O_T_EE[14]])

    current_position = np.array([robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                    robot_state.O_T_EE[14]])

    if time > 0 and (np.linalg.norm(current_position - initial_position)) > 0.01:
        raise RuntimeError("Aborting; too far away from starting pose!")

    # get state variables
    jacobian_array = model.zeroJacobian(Frame.kEndEffector, robot_state)

    jacobian = np.array(jacobian_array).reshape(7, 6)
    tau_measured = np.copy(robot_state.tau_J).reshape(7, 1)
    gravity = np.copy(gravity_array).reshape(7, 1)
    

    tau_d = np.zeros(7,dtype=float)
    desired_force_torque = np.zeros(6, dtype=float)
    tau_cmd = np.zeros(7,dtype=float)
    tau_ext = np.zeros(7,dtype=float)

    desired_force_torque[2] = desired_mass * -9.81
    tau_ext = tau_measured - gravity - initial_tau_ext
    tau_d = jacobian @ desired_force_torque
    tau_error_integral = tau_error_integral + period.toSec() * (tau_d.T - tau_ext.T)

    # print("jacobian_array: ", jacobian_array)
    # print("jacobian: ", jacobian)
    # print("desired_force_torque: ", desired_force_torque)
    # print("tau_d: ", tau_d)
    # print("tau_error_integral: ", tau_error_integral)
    # FF + PI control
    tau_cmd = tau_d + k_p * (tau_d.T - tau_ext.T) + k_i * tau_error_integral

    # Smoothly update the mass to reach the desired target value
    desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass

    # print("tau_cmd:", tau_cmd)
    # print("tau_cmd[0]:", tau_cmd[0])
    tau_d_array = tau_cmd[0].tolist()
    print("tau_d_array:", tau_d_array)

    torques_obj = Torques(tau_d_array)

    return torques_obj

while True:
    franka_api_obj.robot_control(torques_handle = torques_controller)

