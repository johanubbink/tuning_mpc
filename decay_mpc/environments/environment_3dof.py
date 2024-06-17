import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import casadi as ca


class environment_3dof:

    def __init__(self) -> None:
        self.num_dof = 3

        # creates the forward kinematics
        self.forward_kin = self._forward_kin_factory()

        # define initial conditions
        self.q0 = np.array([0.0, np.pi/2, -np.pi/4])
        self.dq0 = np.array([0.0, 0.0, 0.0])
        self.t0 = np.array([0.0])

        self.q_upper = ca.MX([1, np.pi+0.01,np.pi + 0.01])
        self.q_lower = ca.MX([-1, -np.pi-0.01, -np.pi - 0.01])

        self.dq_lim = 0.5
        self.ddq_lim = 1.5


    def _forward_kin_factory(self,L1 = 1, L2=1) -> ca.Function:
        """
        Generates a casadi function for the forward kinematics
        """

        q0 = ca.SX.sym("q0")
        q1 = ca.SX.sym("q1")
        q2 = ca.SX.sym("q2")

        q = ca.vertcat(q0,q1,q2)

        x0 = q0
        y0 = 0
        p0 = ca.vertcat(x0, y0)
        x1 = x0 + L1 * ca.cos(q1)
        y1 = y0 + L1 * ca.sin(q1)
        p1 = ca.vertcat(x1, y1)
        x2 = x1 + L2 * ca.cos(q1 + q2)
        y2 = y1 + L2 * ca.sin(q1 + q2)
        p2 = ca.vertcat(x2, y2)

        return ca.Function("forward_kin", [q],[p0,p1,p2], ["q"], ["p0", "p1", "p2"])


    def trajectory(self,t):
        '''
        Circular trajectory that the robot should follow.    
        '''
        # p = 6
        o = [0.0, 1.5]

        # return ca.vertcat(o[0] + 1.3*r*ca.cos(2*np.pi*t/p), o[1] - r*ca.sin(2*np.pi*t/p))
        # return ca.vertcat(o[0] + 1*ca.cos(2*np.pi*(t)/p), o[1])
        return ca.vertcat(o[0] + ca.cos(t), o[1])



    def animate(self,q: list, t: np.array, dt=float, title=""):
        '''
        Function to animate 2 link robot arm.

        Parameters:
        - q (list): List of joint angles for each time step. A list of arrays or lists
        - t (np.array): Array of time steps.
        - dt (float): Time interval between frames (default: 1.0).
        - title (str): Title for the animation (default: "").

        Returns:
        - None

        '''

        p0, p1, p2 = self.forward_kin(q[0])

        p0 = p0.full().ravel()
        p1 = p1.full().ravel()
        p2 = p2.full().ravel()

        target = self.trajectory(t[0]).full().ravel()

        fig, ax = plt.subplots()

        link0, = ax.plot([-1,1],[0,0])
        link1, = ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'r-', linewidth=2)
        link2, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', linewidth=2)
        target_point, = ax.plot([target[0]], [target[1]],"go")  # green circle for target

        

        def update(i):
            p0, p1, p2 = self.forward_kin(q[i])
            p0 = p0.full().ravel()
            p1 = p1.full().ravel()
            p2 = p2.full().ravel()
            link1.set_data([p0[0], p1[0]], [p0[1], p1[1]])
            link2.set_data([p1[0], p2[0]], [p1[1], p2[1]])

            target = self.trajectory(t[i]).full().ravel()
            target_point.set_data([target[0]], [target[1]])
            return link1, link2, target_point


        ani = animation.FuncAnimation(
            fig, update, frames=t.ravel().shape[0], interval=dt, blit=True)


        ax.set_xlim(-2, 2)
        ax.set_ylim(-0.5, 2)
        ax.set_aspect('equal')  # Set the aspect ratio to 1
        plt.title(title)
        

        return ani




if __name__ == "__main__":

    # create a time vector
    dt = 0.01
    t = np.arange(0, 5, dt)

    q0 = np.sin(0.8*t)[:, np.newaxis]*0.2 + 0
    q1 = np.sin(0.8*t)[:, np.newaxis]*0.2 + np.pi/2
    q2 = np.cos(0.8*t)[:, np.newaxis]*0.2 + 0
    q = np.hstack((q0,q1, q2))

    env = environment_3dof()

    env.animate(q, t, dt)
