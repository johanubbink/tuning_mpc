from decay_mpc.utils.simulator import perform_sim

from decay_mpc.environments.environment_3dof import environment_3dof
import numpy as np

from matplotlib import pyplot as plt
import os

# setup plotting for the paper
plt.rcParams["text.usetex"] = True
plt.rcParams["font.family"] = "serif"
plt.rcParams['font.size'] = 11
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10


# specify parameters
ts = 0.01


def to_numpy(x):
    return np.array(x.to_list()).reshape((-1, 2))


# load a simulation environment
env = environment_3dof()

show_animation = True

current_dir = os.path.dirname(os.path.abspath(__file__))
save_dir = os.path.join(current_dir, "figures")
os.makedirs(save_dir, exist_ok=True)


def plot_error(error_data, t, legend_title):
    plt.figure(figsize=(5, 2.))
    for key, item in error_data.items():
        # plt.plot(t, item[:, 1], label=r"$\mu=$"+key)
        plt.plot(t, item[:, 0], label=key)

    plt.xlabel('Time [s]')
    plt.ylabel('Error in x [m]')
    plt.grid()
    plt.legend(loc="lower right", ncol=1, title=legend_title, handlelength=1)
    plt.tight_layout()

    return plt.gca()



def objective_function_ee_accel(internal_dynamics):
    from decay_mpc.controllers import mpc_penalize_e_ddot
    N = 3

    # create a controller

    error_data = {}
    p2_data = {}

    # create an environment
    env = environment_3dof()
    # for mu in [1e-5, 1e-4, 1e-3, 1e-2]:
    powers = [-5, -4, -3, -2]
    for power in powers:
        mu = 10**power
        controller = mpc_penalize_e_ddot.generate_controller(
            ts, N, env =env, mu=mu)
        simulated_data = perform_sim(
            controller, env, 100, 5, internal_dynamics)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        mu_str = r'$10^{{{:.0f}}}$'.format(power)

        error_data[mu_str] = e
        p2_data[mu_str] = to_numpy(simulated_data["p2"])

        if show_animation:
            ani = env.animate(
                simulated_data["q"].to_list(), t, 0.01, "mu = " + mu_str)

    if internal_dynamics:
        save_path = os.path.join(save_dir, "objective_a_with_dynamics.pdf")
    else:
        save_path = os.path.join(save_dir, "objective_a_perfect.pdf")

    plot = plot_error(error_data, t, r"$\mu$")
    # plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    plt.show()

print("Objective function A with perfect simulation")
objective_function_ee_accel(internal_dynamics=True)
