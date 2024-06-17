from tqdm import tqdm
import casadi as ca
import numpy as np
import pandas as pd

def dynamics(x, u, dt, k):
    xdot = -k*(x-u)
    return x + dt*xdot


def perform_sim(controller, env, freq=100, T=5, internal_dynamics = False):


    # simulation loop
    dt = 1/freq
    t_vec = np.arange(0, T + dt, dt)

    log_list = []

    # set the initial conditions
    q = env.q0
    dq = env.dq0
    t = env.t0

    # starting acceleration of the robot
    ddq = 0

    if internal_dynamics:
        internal_k = 15

    warmstart = 0

    for t_step in tqdm(t_vec):

        ddq_cmd, error, p2, warmstart = controller(q, dq, t, warmstart)

        log = {"t": t, "q": q, "dq": dq, "ddq": ddq,
               "ddq_cmd": ddq_cmd, "e": error, "p2": p2}

        # convert all the outputs to numpy arrays
        for k, v in log.items():
            if isinstance(v, ca.DM):
                log[k] = np.array(v.full()).ravel()

        # log the output
        log_list.append(log)

        if internal_dynamics:
            ddq = dynamics(ddq, ddq_cmd, dt, internal_k)
        else:
            ddq = ddq_cmd

        # integrate joints
        q = q + dt * dq
        dq = dq + dt * ddq
        t = t + dt

    # print (pd.DataFrame(log_list).head())

    simulated_data = pd.DataFrame(log_list)
    return simulated_data
