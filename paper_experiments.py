from decay_mpc.utils.simulator import perform_sim

from decay_mpc.environments.environment_3dof import environment_3dof
import numpy as np

from matplotlib import pyplot as plt
import os
import matplotlib
# setup plotting for the paper
# plt.rcParams["text.usetex"] = True
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
save_animation = True
show_plot = False
graph_extension = ".pdf"
tight_layout = False

current_dir = os.path.dirname(os.path.abspath(__file__))
figure_dir = os.path.join(current_dir, "figures")
animation_dir = os.path.join(current_dir, "animations")
os.makedirs(figure_dir, exist_ok=True)
os.makedirs(animation_dir, exist_ok=True)



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


def objective_function_a(internal_dynamics):
    from decay_mpc.controllers import vanilla_mpc_controller

    N = 40
    mu = 1e-4

    # create a controller

    error_data = {}
    p2_data = {}

    # create an environment
    env = environment_3dof(constraints=False)
    # for mu in [1e-5, 1e-4, 1e-3, 1e-2]:
    powers = [-5, -4, -3, -2]
    for power in powers:
        mu = 10**power
        controller = vanilla_mpc_controller.generate_controller(
            ts, N, env, mu=mu, lam=0)
        simulated_data = perform_sim(
            controller, env, 100, 5, internal_dynamics)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        mu_str = r'$10^{{{:.0f}}}$'.format(power)

        error_data[mu_str] = e
        p2_data[mu_str] = to_numpy(simulated_data["p2"])

        ani = env.animate(simulated_data["q"].to_list()[::5], t[::5],
                          0.01*5*1000, "mu=" + str(mu) + ", N= " + str(N))

        if show_animation:
            plt.show()
        if save_animation:
            title = "objective_a_mu_" + str(power)
            if internal_dynamics:
                title = title + "_with_dynamics"
            else:
                title = title + "_perfect"
            save_path = os.path.join(animation_dir, title + ".gif")
            ani.save(save_path, writer="ffmpeg")
        
        plt.close("all")



    if internal_dynamics:
        save_path = os.path.join(figure_dir, "objective_a_with_dynamics" + graph_extension)
    else:
        save_path = os.path.join(figure_dir, "objective_a_perfect" + graph_extension)

    plot = plot_error(error_data, t, r"$\mu$")
    if tight_layout:
        plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    else:
        plt.savefig(save_path)
    if show_plot:
        plt.show()



def objective_function_b(internal_dynamics):
    from decay_mpc.controllers import vanilla_mpc_controller
    N = 40
    mu = 1e-4

    # create a controller
    error_data = {}

    # create an environment
    env = environment_3dof(constraints=False)

    for N in [100, 30, 10, 2]:
        controller = vanilla_mpc_controller.generate_controller(
            ts, N, env, mu=mu, lam=1e-1)
        
        simulated_data = perform_sim(
            controller, env, 100, 5, internal_dynamics)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        error_data[str(N)] = e


        ani = env.animate(simulated_data["q"].to_list()[::5], t[::5],
                          0.01*5*1000, "mu=" + str(mu) + ", N= " + str(N))

        if show_animation:
            plt.show()
        if save_animation:
            title = "objective_b_N_" + str(N)
            if internal_dynamics:
                title = title + "_with_dynamics"
            else:
                title = title + "_perfect"
            save_path = os.path.join(animation_dir, title + ".gif")
            ani.save(save_path, writer="ffmpeg")

        plt.close("all")
  


    if internal_dynamics:
        save_path = os.path.join(figure_dir, "objective_b_with_dynamics" + graph_extension)
    else:
        save_path = os.path.join(figure_dir, "objective_b_perfect" + graph_extension)

    plot = plot_error(error_data, t, r"$N$")   
    if tight_layout:
        plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    else:
        plt.savefig(save_path)
    if show_plot:
        plt.show()



def objective_function_c(internal_dynamics, with_constraints):
    from decay_mpc.controllers import decay_based_mpc_2
    N = 40
    mu = 1e-5
    K = 2

    # create a controller
    error_data = {}

    # create an environment
    env = environment_3dof(constraints=with_constraints)

    for N in [100, 30, 10, 2]:
        controller = decay_based_mpc_2.generate_controller(
            ts, N, env, K=K, mu=mu, with_constraints=with_constraints)

        simulated_data = perform_sim(
            controller, env, 100, 5, internal_dynamics)

        e = to_numpy(simulated_data["e"])
        t = simulated_data["t"].to_numpy()

        # log the errors
        error_data[str(N)] = e

        
        ani = env.animate(simulated_data["q"].to_list()[::5], t[::5],
                        0.01*5*1000, "mu=" + str(mu) + ", N= " + str(N))
        
        if show_animation:
            plt.show()
        if save_animation:
            title = "objective_c_N_" + str(N)
            if internal_dynamics:
                title = title + "_with_dynamics"
            else:
                title = title + "_perfect"
            if with_constraints:
                title = title + "_with_constraints"
            save_path = os.path.join(animation_dir, title + ".gif")
            ani.save(save_path, writer="ffmpeg")
        
        plt.close("all")

            
    title = "objective_c"

    if internal_dynamics:
        title = title + "_with_dynamics"
    else:
        title = title + "_perfect"

    if with_constraints:
        title = title + "_with_constraints"
    else:
        title = title + "_no_constraints"
    save_path = os.path.join(figure_dir, title + graph_extension)

    plot = plot_error(error_data, t, r"$N$")

    item = error_data[str(N)]
    plt.plot([1/K], [0.37 * item[0, 0]], "o", c="black")
    plt.text(1/K + 0.09, 0.37 * item[0, 0] - 0.05,
             r'$[\alpha^{-1}, 0.37 e_0]$', c="black")
    
    if tight_layout:
        plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
    else:
        plt.savefig(save_path)
    if show_plot:
        plt.show()

print("Objective function A with perfect simulation")
objective_function_a(internal_dynamics=False)

print("Objective function A with internal dynamics simulation")
objective_function_a(internal_dynamics=True)

print("Objective function B")
objective_function_b(internal_dynamics=True)

print("Objective function C")
objective_function_c(internal_dynamics=True, with_constraints= False)

print("Objective function C - with constraints")
objective_function_c(internal_dynamics=True, with_constraints=True)
