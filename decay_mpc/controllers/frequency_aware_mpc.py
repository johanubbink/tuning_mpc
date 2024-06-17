import fatropy.spectool as sp

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm
import casadi as ca
import scipy.signal as signal


from decay_mpc.environments.environment_3dof import environment_3dof
import os


def generate_shaping_filter(alpha, beta) -> signal.StateSpace:
    '''
    computes the state space representation of a given filter
    
    returns the continuous state space model
    A, B, C, D
    '''

    numerator = [beta, 1]
    denominator = [alpha, 1]

    # Create the transfer function object
    tf = signal.TransferFunction(numerator, denominator)

    ss = tf.to_ss()

    return ss


def generate_controller(ts, horizon_len, mu, env: environment_3dof, **kwargs):
    ocp = sp.Ocp()

    #  define the robot dynamics
    q = ocp.state(env.num_dof)
    dq = ocp.state(env.num_dof)
    u = ocp.control(env.num_dof)

    # define a time variable
    t = ocp.state()
    t0 = ocp.parameter()

    q0 = ocp.parameter(env.num_dof)
    dq0 = ocp.parameter(env.num_dof)

    # define a frequency filter
    beta = kwargs['beta']
    filter = generate_shaping_filter(0.1*beta, beta)
    # set the system dynamics
    xs = ocp.state(env.num_dof)
    xs0 = ocp.parameter(env.num_dof)
    print(filter.A)
    xsdot = filter.A*xs + filter.B*u
    reg = filter.C*xs + filter.D*u

    stage = ocp.new_stage(horizon_len)
    stage.set_next(xs, xs + ts*xsdot)

    # set the dynamics
    stage.set_next(q, q + ts*dq)
    stage.set_next(dq, dq + ts*u)
    stage.set_next(t, t + ts)

    # constrain the initial state
    stage.at_t0().subject_to(q == q0)
    stage.at_t0().subject_to(dq == dq0)
    stage.at_t0().subject_to(t == t0)

    # all the states
    # x = ca.vertcat(q, dq, t)
    # x_dot = ca.vertcat(dq, v, 1)

    # robot_kinematics = forward_kin_factory()
    ee = env.forward_kin(q)[-1]

    error = ee - env.trajectory(t)
    # error_dot = ca.jtimes(error, x, x_dot)

    # eps = error_dot + K * error

    # specify the objective
    stage.add_objective(mu*reg.T@reg, sp.t0, sp.mid)
    # stage.add_objective(1e-1*error_dot.T@error_dot, sp.mid, sp.tf)
    stage.add_objective(error.T@error, sp.mid, sp.tf)

    ocp.solver('fatrop', {"jit": False, "error_on_fail": True, "post_expand": True}, {
               "print_level": 0, "warm_start_init_point": True, "tol": 1e-6})

    return ocp.to_function("ocp", [q0, dq0, t0, xs0, ocp.all_variables()], [ocp.at_t0(u), ocp.at_t0(error), ocp.at_t0(ee), ocp.at_t0(xsdot), ocp.all_variables()])


def perform_sim(ts, horizon_len, mu, env: environment_3dof, freq=100, T=5, **kwargs):

    controller = generate_controller(ts, horizon_len, mu, env, **kwargs)

    # simulation loop
    dt = 1/freq
    t_vec = np.arange(0, T + dt, dt)

    log_list = []

    # set the initial conditions
    q = env.q0
    dq = env.dq0
    t = env.t0
    xs = 0
    ddq = 0
    internal_k = 15
    warmstart = 0

    for t_step in tqdm(t_vec):

        ddq_cmd, error, p2, xsdot,  warmstart = controller(
            q, dq, t, xs, warmstart)

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
        xs = xs + dt*xsdot
        # print ("xs",xs)
    # print (pd.DataFrame(log_list).head())

    simulated_data = pd.DataFrame(log_list)
    return simulated_data


def to_numpy(x):
    return np.array(x.to_list()).reshape((-1, 2))


def dynamics(x, u, dt, k):
    xdot = -k*(x-u)
    return x + dt*xdot


if __name__ == "__main__":

    ts = 0.01
    mu = 1e-3
    error_data = {}

    # create an environment
    env = environment_3dof()
    beta = 1/2

    # for N in [25, 10, 5, 2]:
    for N in [100, 40, 25, 15, 10]:

        simulated_data = perform_sim(ts, N, mu, env, T=7, beta=beta)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        error_data[str(N)] = e

        env.animate(simulated_data["q"].to_list(), t,
                    0.01, "mu=" + str(mu) + ", N= " + str(N))

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
    save_path = os.path.join(
        save_dir, "objective_frequency_aware_dynamics.pdf")
    # plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    plt.show()

    # ts = 0.01
    # N = 50

    # error_data = {}
    # p2_data = {}

    # # create an environment
    # env = environment_3dof()

    # # for mu in [1e-5, 1e-4, 1e-3, 1e-2]:
    # mu = 1e-4
    # for beta_inv in [1, 5, 10, 20]:
    #     simulated_data = perform_sim(ts, N, mu, env, T=7, beta=1/beta_inv)

    #     e = to_numpy(simulated_data["e"])
    #     t = simulated_data["t"].to_numpy()

    #     # log the errors
    #     # mu_str = r'$10^{{{:.0f}}}$'.format(beta)
    #     mu_str = str(beta_inv)
    #     error_data[mu_str] = e
    #     p2_data[mu_str] = to_numpy(simulated_data["p2"])

    #     # env.animate(simulated_data["q"].to_list(), t, 0.01, "mu = " + mu_str)

    #     # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    #     # for key, item in error_data.items():
    #     #     ax1.plot(t, item[:, 0], label=key)
    #     #     ax2.plot(t, item[:, 1], label=key)
    #     # ax1.set_xlabel('Time')
    #     # ax1.set_ylabel('Error in x')
    #     # # ax1.legend()
    #     # ax2.set_xlabel('Time')
    #     # ax2.set_ylabel('Error in y')
    #     # ax2.legend()
    #     # plt.show()

    # # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

    # plt.rcParams["text.usetex"] = True
    # plt.rcParams["font.family"] = "serif"
    # plt.rcParams['font.size'] = 12
    # plt.rcParams['xtick.labelsize'] = 10
    # plt.rcParams['ytick.labelsize'] = 10

    # plt.figure(figsize=(5, 2.2))
    # for key, item in error_data.items():
    #     # plt.plot(t, item[:, 1], label=r"$\mu=$"+key)
    #     plt.plot(t, item[:, 0], label=key)

    # plt.xlabel('Time [t]')
    # plt.ylabel('Error in x [m]')
    # plt.grid()
    # plt.legend(loc="lower right", ncol=1, title=r"$\beta^{-1}$", handlelength=1)
    # plt.tight_layout()
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # save_dir = os.path.join(current_dir, "figures")
    # os.makedirs(save_dir, exist_ok=True)
    # save_path = os.path.join(save_dir, "frequency_aware_beta.pdf")
    # plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    # plt.show()
