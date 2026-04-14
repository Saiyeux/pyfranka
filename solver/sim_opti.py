import casadi as ca
from casadi.casadi import reshape
import numpy as np
import time
from functions import *

def shift_movement(T, x0, u, x, f):
    #- state transfer function
    f_value = f(x0, u[0])
    #- relation between two successive state
    x1 = x0 + T * f_value
    
    #- for next guess, apply left shift for U, X then copy the last column
    u_guess = np.concatenate((u[1:], u[-1:]), axis=0)
    x_guess = np.concatenate((x[1:], x[-1:]), axis=0)

    return x1, u_guess, x_guess


#. config
predict_horizon = 15  # prediction length
time_step = 0.1  # period
omega_max = 5  # angle speed: deg/s
sim_time = 10

#. limit
LOWER_J = [-120, -100, -180, -150, -180, -30, -100]
UPPER_J = [60, 30, 180, 10, 180, 100, 100]

#. variable
opti = ca.Opti()
opt_controls = opti.variable(predict_horizon, 7)
opt_states = opti.variable(predict_horizon + 1, 7)

n_states = 7
q = opt_states
n_controls = 7
dq = opt_controls

#. parameters
opt_x0 = opti.parameter(7)
opt_xs = opti.parameter(7)

#. create model
f = lambda q_, dq_: dq_
f_np = lambda q_, dq_: np.array(dq_).reshape(-1, 1)

#. init condition
opti.subject_to(opt_states[0, :] == opt_x0.T)
for i in range(predict_horizon):
    x_next = opt_states[i, :] + f(opt_states[i, :],
                                  opt_controls[i, :]) * time_step
    opti.subject_to(opt_states[i + 1, :] == x_next)

#. cost weights and function
Q = np.eye(7)
Qn = deepcopy(Q) * 10
R = np.eye(7)

obj = 0
for i in range(predict_horizon + 1):
    if i < predict_horizon:
        obj = obj + ca.mtimes([
            (opt_states[i, :] - opt_xs.T), Q, (opt_states[i, :] - opt_xs.T).T
        ]) + ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T])
    else:
        obj = obj + ca.mtimes([(opt_states[i, :] - opt_xs.T), Qn,
                               (opt_states[i, :] - opt_xs.T).T])

opti.minimize(obj)

#. boundary and control conditions
opti.subject_to(opti.bounded(-omega_max, dq, omega_max))
for i in range(n_states):
    opti.subject_to(opti.bounded(LOWER_J[i], q[:, i], UPPER_J[i]))
# for i in range(predict_horizon + 1):
#     opti.subject_to(opti.bounded(LOWER_J, q[i, :].T, UPPER_J))

#. opt_settings
opts_setting = {
    'ipopt.max_iter': 50,
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.acceptable_tol': 1e-3,
    'ipopt.acceptable_obj_change_tol': 1e-3
}
opti.solver('ipopt', opts_setting)

#. simulation
def simulation(q0, qs):
    
    #- record
    record_start_time = time.time()
    x_data = []
    
    x0 = q0.reshape(-1, 1)
    xs = qs.reshape(-1, 1)
    opti.set_value(opt_xs, xs)
    
    #. parameters intialization
    mpc_iter = 0
    U_guess = np.zeros((predict_horizon, n_states))
    X_guess = np.zeros((predict_horizon + 1, n_states))  

    #. start mpc loop
    while (np.linalg.norm(x0 - xs) > 0.5 and mpc_iter - sim_time / time_step < 0.0):
        #- update parameter 
        opti.set_value(opt_x0, x0)
        opti.set_initial(opt_controls, U_guess)
        opti.set_initial(opt_states, X_guess)

        #- solve nlp
        res = opti.solve()
        U_result = res.value(opt_controls)
        X_result = res.value(opt_states)

        #- update state and guess
        x0, U_guess, X_guess = shift_movement(time_step, x0, U_result, X_result, f_np)

        #- save state
        mpc_iter = mpc_iter + 1
        x_data.append(x0)
        
    record_end_time = time.time()
    print('average time: ', (record_end_time - record_start_time) / mpc_iter)

    return x_data

q0 = np.array([0, 0, 0, -90, 0, 0, 0], dtype=float)
q1 = np.array([10, 20, 30, -50, 40, 20, 10], dtype=float)
x_data = simulation(q0, q1)
draw_pic(x_data)