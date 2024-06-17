import fatropy.spectool as sp

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import casadi as ca


from decay_mpc.environments.environment_3dof import environment_3dof
import os


def generate_controller(ts, horizon_len, mu, env: environment_3dof):
    ocp = sp.Ocp()

    #  define the robot dynamics
    q = ocp.state(env.num_dof)
    dq = ocp.state(env.num_dof)
    ddq = ocp.control(env.num_dof)

    # define a time variable
    t = ocp.state()
    t0 = ocp.parameter()

    q0 = ocp.parameter(env.num_dof)
    dq0 = ocp.parameter(env.num_dof)

    stage = ocp.new_stage(horizon_len)

    # all the states
    x = ca.vertcat(q, dq, t)
    x_dot = ca.vertcat(dq, ddq, 1)

    # set the dynamics
    stage.set_next(q, q + ts*dq)
    stage.set_next(dq, dq + ts*ddq)
    stage.set_next(t, t + ts)

    # constrain the initial state
    stage.at_t0().subject_to(q == q0)
    stage.at_t0().subject_to(dq == dq0)
    stage.at_t0().subject_to(t == t0)

    # robot_kinematics = forward_kin_factory()
    ee = env.forward_kin(q)[-1]

    error = ee - env.trajectory(t)
    error_dot = ca.jtimes(error, x, x_dot)
    error_ddot = ca.jtimes(error_dot, x, x_dot)


    # specify the objective
    stage.add_objective(mu*ddq.T@ddq, sp.t0, sp.mid)
    stage.add_objective(1e-1*error_ddot.T@error_ddot, sp.mid)
    stage.add_objective(error.T@error, sp.mid)

    ocp.solver('fatrop', {"jit": False, "error_on_fail": True, "post_expand": True}, {
               "mu_init": 1e-1, "print_level": 0})

    return ocp.to_function("ocp", [q0, dq0, t0, ocp.all_variables()], [ocp.at_t0(ddq), ocp.at_t0(error), ocp.at_t0(ee), ocp.all_variables()])


def dynamics(x, u, dt, k):
    xdot = -k*(x-u)
    return x + dt*xdot


def perform_sim(ts, horizon_len, mu, env: environment_3dof, freq=100, T=5):

    controller = generate_controller(ts, horizon_len, mu, env)

    # simulation loop
    dt = 1/freq
    t_vec = np.arange(0, T + dt, dt)

    log_list = []

    # set the initial conditions
    q = env.q0
    dq = env.dq0
    t = env.t0

    ddq = 0
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

        ddq = dynamics(ddq, ddq_cmd, dt, internal_k)

        # integrate joints
        q = q + dt * dq
        dq = dq + dt * ddq
        t = t + dt

    # print (pd.DataFrame(log_list).head())

    simulated_data = pd.DataFrame(log_list)
    return simulated_data


def to_numpy(x):
    return np.array(x.to_list()).reshape((-1, 2))


if __name__ == "__main__":

    ts = 0.01
    mu = 1e-4
    error_data = {}

    # create an environment
    env = environment_3dof()

    # for N in [25, 10, 5, 2]:
    for N in [100, 30, 10, 2]:

        simulated_data = perform_sim(ts, N, mu, env, T=7)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        error_data[str(N)] = e

        env.animate(simulated_data["q"].to_list(), t, 0.01, "mu=" +str(mu)+ ", N= " + str(N))

    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    plt.rcParams["text.usetex"] = True
    plt.rcParams["font.family"] = "serif"
    plt.rcParams['font.size'] = 12
    plt.rcParams['xtick.labelsize'] = 10
    plt.rcParams['ytick.labelsize'] = 10

    plt.figure(figsize=(5, 2.2))
    for key, item in error_data.items():
        plt.plot(t, item[:, 0], label=key)
    plt.xlabel('Time [t]')
    plt.ylabel('Error in x [m]')
    plt.grid()
    plt.legend(loc="lower right", ncol=1, title=r"$N$", handlelength=1)
    plt.tight_layout()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(current_dir, "figures")
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, "objective_2_dynamics.pdf")
    # plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    plt.show()
