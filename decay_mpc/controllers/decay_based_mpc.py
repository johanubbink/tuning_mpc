import fatropy.spectool as sp

import matplotlib.pyplot as plt
import numpy as np
import casadi as ca


from decay_mpc.environments.environment_3dof import environment_3dof
import os


def generate_controller(ts, horizon_len, env: environment_3dof, with_constraints= True, ** kwargs):
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

    eps = error_dot + kwargs["K"] * error

    # specify the objective
    stage.add_objective(kwargs["mu"]*ddq.T@ddq, sp.t0, sp.mid)
    # stage.add_objective(kwargs["lam"]*error_dot.T@error_dot, sp.mid)
    stage.add_objective(eps.T@eps, sp.mid)

    # add the constraints
    if with_constraints:
        stage.subject_to((env.q_lower <= ((1/kwargs["K"])*dq + q))
                            <= env.q_upper, sp.mid, sp.tf)
        stage.subject_to((-env.dq_lim <= ((1/kwargs["K"])*ddq + dq))
                        <= env.dq_lim, sp.t0, sp.mid, sp.tf)
        stage.subject_to((-env.ddq_lim <= ddq) <= env.ddq_lim, sp.t0, sp.mid)


    ocp.solver('fatrop', {"jit": False, "error_on_fail": False, "post_expand": True}, {"print_level": 0})

    return ocp.to_function("ocp", [q0, dq0, t0, ocp.all_variables()], [ocp.at_t0(ddq), ocp.at_t0(error), ocp.at_t0(ee), ocp.all_variables()])



def to_numpy(x):
    return np.array(x.to_list()).reshape((-1,2))



if __name__ == "__main__":

    from decay_mpc.utils.simulator import perform_sim
    N = 40
    mu = 1e-5
    K = 2
    ts = 1/100
    internal_dynamics = False

    # create a controller
    error_data = {}

    # create an environment
    env = environment_3dof()


    controller = generate_controller(
        ts, N, env, K=K, mu=mu, with_constraints=False)

    simulated_data = perform_sim(
        controller, env, 100, 5, internal_dynamics)

    e = to_numpy(simulated_data["e"])
    t = simulated_data["t"].to_numpy()



    ani = env.animate(simulated_data["q"].to_list()[::4], t[::4],
                0.01*4*1000, "mu=" + str(mu) + ", N= " + str(N))

    plt.show()
    ani.save("animations/mymovie.mp4", writer="ffmpeg")
    ani.save("animations/mymovie.gif", writer="ffmpeg")

    with open("animations/myvideo.html", "w") as f:
        print(ani.to_html5_video(), file=f)

    with open("animations/myvideo_js.html", "w") as f:
        print(ani.to_jshtml(), file=f)
    plt.close("all")


    # ani.save(filename="animations/html_example.html", writer="html")

    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))