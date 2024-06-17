import fatropy.spectool as sp

import matplotlib.pyplot as plt
import numpy as np
import casadi as ca


from decay_mpc.environments.environment_3dof import environment_3dof
import os


def generate_controller(ts, horizon_len, env :environment_3dof, **kwargs):
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
    x = ca.vertcat(q,dq,t)
    x_dot = ca.vertcat(dq,ddq, 1)

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

    # specify the objective
    stage.add_objective(kwargs["mu"]*ddq.T@ddq, sp.t0, sp.mid)
    stage.add_objective(kwargs["lam"]*error_dot.T@error_dot, sp.mid)
    stage.add_objective(error.T@error, sp.mid)

    ocp.solver('fatrop', {"jit": False, "error_on_fail": False, "post_expand": True}, {"print_level": 0})

    return ocp.to_function("ocp", [q0, dq0, t0, ocp.all_variables()], [ocp.at_t0(ddq), ocp.at_t0(error), ocp.at_t0(ee), ocp.all_variables()])



def to_numpy(x):
    return np.array(x.to_list()).reshape((-1,2))



if __name__ == "__main__":

    from decay_mpc.utils.simulator import perform_sim

    # specify parameters
    ts = 0.01
    N = 40
    mu = 1e-4

    # define an environments
    env = environment_3dof()

    # create a controller
    

    error_data = {}
    p2_data = {}

    # create an environment
    env = environment_3dof()
    # for mu in [1e-5, 1e-4, 1e-3, 1e-2]:
    powers = [-4]
    for power in powers:
        mu = 10**power
        controller = generate_controller(ts, N, env, mu=mu, lam=1e-4)
        simulated_data = perform_sim(controller, env, 100, 5)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        mu_str = r'$10^{{{:.0f}}}$'.format(power)

        error_data[mu_str] = e
        p2_data[mu_str] = to_numpy(simulated_data["p2"])

        env.animate(simulated_data["q"].to_list(), t, 0.01, "mu = " + mu_str)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
        for key, item in error_data.items():
            ax1.plot(t, item[:, 0], label=key)
            ax2.plot(t, item[:, 1], label=key)
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Error in x')
        # ax1.legend()
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Error in y')
        ax2.legend()
        plt.show()
    
