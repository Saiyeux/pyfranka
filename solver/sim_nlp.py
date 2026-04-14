import casadi as ca
import numpy as np
import time
from functions import *

#. config
predict_horizon = 15  # prediction length
time_step = 0.1  # period
omega_max = 5  # angle speed: deg/s
sim_time = 10

#. limit
LOWER_J = [-120, -100, -180, -150, -180, -30, -100]
UPPER_J = [60, 30, 180, 10, 180, 100, 100]


#. variable
q = ca.SX.sym('q', 7, 1)
states = q
n_states = states.size()[0]

dq = ca.SX.sym('dq', 7, 1)
controls = dq
n_controls = controls.size()[0]
divide = predict_horizon * n_controls


#.  constraint
tol = 0
lbg = -tol
ubg = tol
lbx = []
ubx = []

for _ in range(predict_horizon):
    #- constraint U
    for i in range(7):
        lbx.append(-omega_max)
    for i in range(7):
        ubx.append(omega_max)
for _ in range(
        predict_horizon +
        1):  # note that this is different with the method using structure
    #- constraint X
    for i in range(7):
        lbx.append(LOWER_J[i])
    for i in range(7):
        ubx.append(UPPER_J[i])

#. solver generation
solver, f = generate_solver_nlp(q, dq, predict_horizon, time_step)


def simulation(q0, qs):
    
    record_start_time = time.time()
    x0 = q0.reshape(-1, 1)
    xs = qs.reshape(-1, 1)
    x_data = []
    #. parameters intialization
    mpc_iter = 0
    U_guess = np.zeros((n_states , predict_horizon))
    X_guess = np.zeros((n_states, predict_horizon + 1))  

    #. start mpc loop
    while (np.linalg.norm(x0 - xs) > 0.5 and mpc_iter - sim_time / time_step < 0.0):
        #- update parameter 'p'
        p_val = np.concatenate((x0, xs))

        #- formulate init control 'U + X'
        init_control = np.concatenate(
            (U_guess.reshape(-1, 1), X_guess.reshape(-1, 1)))

        #- solve nlp
        res = solver(x0=init_control,
                     p=p_val,
                     lbg=lbg,
                     lbx=lbx,
                     ubg=ubg,
                     ubx=ubx)

        estimated_opt = res['x'].full()

        #- result
        U_result = estimated_opt[:divide].reshape(predict_horizon, n_controls)  # (N, n_controls)
        X_result = estimated_opt[divide:].reshape(predict_horizon + 1, n_states)  # (N+1, n_states)

        #- update state and guess
        x0, U_guess, X_guess = shift_movement(time_step, x0, U_result, X_result, f)
        x0 = ca.reshape(x0, -1, 1)
        x0 = x0.full()

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

