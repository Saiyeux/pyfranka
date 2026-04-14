import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

def shift_movement(T, x0, u, x, f):
    #- state transfer function
    f_value = f(x0, u[0, :])
    #- relation between two successive state
    x1 = x0 + T * f_value.full()
    
    #- for next guess, apply left shift for U, X then copy the last column
    u_guess = np.concatenate((u[1:], u[-1:]), axis=0)
    x_guess = np.concatenate((x[1:], x[-1:]), axis=0)

    return x1, u_guess, x_guess

#. solver
def generate_solver_nlp(q, dq, N, T):
    
    states = q
    n_states = states.size()[0]
    controls = dq
    n_controls = controls.size()[0]
    
    A = np.eye(7)
    rhs = ca.vertcat(A @ dq)
    ## function
    f = ca.Function('f', [states, controls], [rhs],
                    ['input_state', 'control_input'], ['rhs'])

    ## for MPC
    U = ca.SX.sym('U', n_controls, N)
    X = ca.SX.sym('X', n_states, N + 1)
    P = ca.SX.sym('P', n_states + n_states) #- angles are free in final state 

    ### define
    Q = np.eye(7)
    Qn = deepcopy(Q) * 10
    R = np.eye(7) 

    #### cost function
    obj = 0  #### cost
    g = []  # equal constrains
    g.append(X[:, 0] - P[:7])
    for i in range(N + 1):
        if i < N:
            obj = obj + ca.mtimes([(X[:7, i] - P[7:]).T, Q, X[:7, i] - P[7:]
                                ]) + ca.mtimes([U[:, i].T, R, U[:, i]])
            x_next_ = f(X[:, i], U[:, i]) * T + X[:, i]
            g.append(X[:, i + 1] - x_next_)
        else:
            obj = obj + ca.mtimes([(X[:7, i] - P[7:]).T, Qn, X[:7, i] - P[7:]])

    
    opt_variables = ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1))

    nlp_prob = {'f': obj, 'x': opt_variables, 'p': P, 'g': ca.vertcat(*g)}
    opts_setting = {
        'ipopt.max_iter': 50,
        'ipopt.print_level': 0,
        'print_time': 0,
        'ipopt.acceptable_tol': 1e-3,
        'ipopt.acceptable_obj_change_tol': 1e-3
    }

    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)
    return solver, f


def draw_pic(data):
    length = len(data)
    x = np.array(data)
    
    x_axis = np.array([i for i in range(length)])
    plt.style.use('seaborn')

    for i in range(7):
        plt.plot(x_axis, x[:, i], label='q' + str(i+1))
    plt.legend()
    plt.show()




    

