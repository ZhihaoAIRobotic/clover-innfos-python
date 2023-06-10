import numpy as np
from math import comb, factorial
from kinematics import fkin, jacobian

class PolynomialGeneratorCart():
    """
    PolynomialGenerator generates a polynomial trajectory given a set of initial and final conditions.
    """

    def __init__(self):
        pass

    def get_constraint_submatrix(self, t, n):
        """
        Get the time constraint submatrix for a given time t

        Args:pi
            t (float): time constraint
            n (int): number of constraints

        Returns:
            (np.array): time constraint submatrix
        """
        if n == 1:
            A = np.array([1, t, t**2, t**3, 0, 0])


        if n == 2:
            A = np.array([[1, t, t**2, t**3, t**4, 0],
                          [0, 1, 2*t, 3*t**2, 4*t**3, 0]])

        if n == 3:
            A = np.array([[1, t, t**2, t**3, t**4, t**5],
                          [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],
                          [0, 0, 2, 6*t, 12*t**2, 20*t**3]])

        return A

    def generate_coefficients(self, q_init, q_final, t_f, t_0=0.0):
        """
        Calculate the polynomial trajectory coefficients from constraints

        Args:
            q_init: list of constraints: [q_init, dq_init, ddq_init]
            q_final: list of constraints: [q_final, dq_final, ddq_final]
            t_f (float): final time
            t_0 (float): start time
        """

        A_0 = self.get_constraint_submatrix(t_0, len(q_init))
        A_f = self.get_constraint_submatrix(t_f, len(q_final))

        M = np.vstack((A_0, A_f))
        n = len(q_init)+len(q_final)
        M = M[:n, :n]
        q = np.hstack((np.array(q_init), np.array(q_final)))

        return np.linalg.inv(M).dot(q)

    def polynomial_from_coefs(self, coefs, from_t, to_t, n):
        """
        Generates discrete polynomial of len(coefs) - 1 degree of size n

        Args:
            coefs: Coefficients of the polynomial to generate
            from_t: Initial time
            to_t: Final time
            n: Number of points to generate

        Returns:
            Array of n points of the polynomial (pose Values)
        """

        ts = np.linspace(from_t, to_t, n)
        poly = 0

        for i, coef in enumerate(coefs):
            poly += coef * ts**i

        return poly

    def dpolynomial_from_coefs(self, d, coefs, from_t, to_t, n):
        """
        Generates dth derivative of a polynomial with coefficients coefs

        Args:
            d : Order of the derivative (1 for dq, 2 for ddq)
            coefs: Coefficients of the polynomial to generate
            from_t: Initial time
            to_t: Final time
            n: Number of points to generate

        Returns:
            Array of n points of the polynomial's dth derivative (pose Velocities or Accelerations)
        """

        ts = np.linspace(from_t, to_t, n)
        poly = 0

        if d >= len(coefs):
            print("Not enough coefficients to generate derivative of order {}".format(d))
            return np.zeros(n)

        for i, coef in enumerate(coefs[d:], d):
            poly += coef * factorial(d) * comb(i, d) * ts**(i-d)

        return poly

    def generate_p2p_trajectory(self, x_init, x_final, t_f, t_0=0.0, n=100):

        """
        Generates a len(q_init)th DOF trajectory given initial and final conditions

        Args:
            x_init (list): Initial conditions (Type: Array)
            x_final (list): Final conditions (Type: Array)
            t_f (float): Final time
            t_0 (float): Initial time
            n (int): Number of points to generate
        """

        x = x_init[0]
        dx = x_init[1]
        ddx = x_init[2]

        if len(x_final) == 1:
            xf = x_final[0]
            x_desired = [xf]

        elif len(x_final) == 2:
            xf = x_final[0]
            dxf = x_final[1]
            x_desired = [xf, dxf]

        elif len(x_final) == 3:
            xf = x_final[0]
            dxf = x_final[1]
            ddxf = x_final[2]
            x_desired = [xf, dxf, ddxf]

        xlist = np.zeros((len(x), n))
        dxlist = np.zeros((len(dx), n))
        ddxlist = np.zeros((len(ddx), n))


        for i in range(len(x)):
            x_in = [x[i], dx[i], ddx[i]]

            if len(x_desired) == 1:
                x_d = [x_desired[0][i]]
            elif len(x_desired) == 2:
                x_d = [x_desired[0][i], x_desired[1][i]]
            elif len(x_desired) == 3:
                x_d = [x_desired[0][i], x_desired[1][i], x_desired[2][i]]

            xlist[i] = self.polynomial_from_coefs(self.generate_coefficients(x_in, x_d, t_f, t_0), t_0, t_f, n)
            dxlist[i] = self.dpolynomial_from_coefs(1, self.generate_coefficients(x_in, x_d, t_f, t_0), t_0, t_f, n)
            ddxlist[i] = self.dpolynomial_from_coefs(2, self.generate_coefficients(x_in, x_d, t_f, t_0), t_0, t_f, n)

        xlist = xlist.T
        dxlist = dxlist.T
        ddxlist = ddxlist.T

        omegalist = np.zeros((len(dxlist), 6))
        for i in range(len(dxlist)):
            sigma, theta, si = xlist[i][3:]
            T = np.array([[0, -np.sin(sigma), np.cos(sigma)*np.sin(theta)],
                          [0, np.cos(sigma), np.sin(sigma)*np.sin(theta)],
                          [1, 0, np.cos(theta)]])

            omegalist[i][3:] = np.matmul(T, dxlist[i][3:])
            omegalist[i][:3] = dxlist[i][:3]

        omegadotlist = np.zeros((len(dxlist), 6))

        for i in range(len(ddxlist)):
            omegadotlist[i][:3] = ddxlist[i][:3]
            sigma, theta, si = xlist[i][3:]
            sigmadot, thetadot, sidot = dxlist[i][3:]
            sigmaddot, thetaddot, siddot = ddxlist[i][3:]

            omegadotlist[i][3] = -sigmadot*thetadot*np.cos(sigma) - np.sin(sigma)*thetaddot + sigmadot*np.sin(sigma)*np.sin(theta)*sidot + thetadot*np.cos(theta)*np.cos(sigma)*sidot + np.cos(sigma)*np.sin(theta)*siddot
            omegadotlist[i][4] = -sigmadot*np.sin(sigma)*thetadot + np.cos(sigma)*thetaddot + sigmadot*np.cos(sigma)*np.sin(theta)*sidot + thetadot*np.cos(theta)*np.sin(sigma)*sidot + np.sin(sigma)*np.sin(theta)*siddot
            omegadotlist[i][5] = sigmaddot - thetadot*np.sin(theta)*sidot + np.cos(theta)*siddot


        return xlist, dxlist, ddxlist

    def generate_trajectory(self, x_init, via_points, from_t, t_list, n_list):
        """
        Generates a len(x_init)th DOF trajectory given initial and final conditions

        Args:
            x_init (list): Initial conditions (Type: Array)
            via_points (list): List of via points (Type: Array)
            from_t (float): Initial time
            t_list (list): List of times for each segment
            n_list (list): List of number of points for each segment
        """
        if len(via_points) != len(t_list):
            print("Number of via points and times do not match the time segments")
            pass

        total = 0
        for i in range(len(n_list)):
            total = total + n_list[i]

        xall_list = np.zeros((total, len(x_init[0])))
        dxall_list = np.zeros((total, len(x_init[0])))
        ddxall_list = np.zeros((total, len(x_init[0])))
        length = 0

        for i in range(len(via_points)):
            if i == 0:
                via0 = via_points[0].reshape(1, 6)
                x_temp, dx_temp, ddx_temp = self.generate_p2p_trajectory(x_init, via0, t_list[0], from_t, n_list[0])
                xall_list[:len(x_temp)] = x_temp
                dxall_list[:len(x_temp)] = dx_temp
                ddxall_list[:len(x_temp)] = ddx_temp
            else:
                length = length + len(x_temp)
                via = via_points[i].reshape(1, 6)
                con_temp = np.array([x_temp[-1], dx_temp[-1], ddx_temp[-1]])
                x_temp, dx_temp, ddx_temp = self.generate_p2p_trajectory(con_temp, via, t_list[i], t_list[i-1], n_list[i])
                xall_list[length: (length + len(x_temp))] = x_temp
                dxall_list[length: (length + len(x_temp))] = dx_temp
                ddxall_list[length: (length + len(x_temp))] = ddx_temp

        return xall_list, dxall_list, ddxall_list

    def plot_all(self, x, dx, ddx):
        """
        Plots the pose values, velocities and accelerations with respect to time

        Args:
            x (np.array): pose values
            dx (np.array): pose velocities
            ddx (np.array): pose accelerations

        Returns:
            A plot of the pose values, velocities and accelerations with respect to time
        """
        x_plot = np.array(x).reshape(6, np.array(x).shape[0])

        for row in x_plot:
            plt.plot(row)
            plt.ylabel("θ (rads)")
            plt.xlabel("Time (s)")
            plt.title("Pose Values")
        plt.show()

        dx_plot = np.array(dx).reshape(6, np.array(dx).shape[0])

        for row in dx_plot:
            plt.plot(row)
            plt.ylabel("d(θ) (rads/second)")
            plt.xlabel("Time (s)")
            plt.title("Pose Velocity")
        plt.show()

        ddx_plot = np.array(ddx).reshape(6, np.array(dx).shape[0])

        for row in ddx_plot:
            plt.plot(row)
            plt.ylabel("d(d(θ)) (rads/second^2)")
            plt.xlabel("Time (s)")
            plt.title("Pose Acceleration")
        plt.show()

    def plot_xi(self, x, dx, ddx, i):
        """
        Plots pose value[i], velocity[i] and acceleration[i] with respect to time

        Args:
            x (np.array): pose values
            dx (np.array): pose velocities
            ddx (np.array): pose accelerations

        Returns:
            A plot of the pose values, velocities and accelerations with respect to time
        """
        x_plot = np.array(x[:, i]).reshape(1, np.array(x).shape[0])

        for row in x_plot:
            plt.plot(row)
            plt.ylabel("m (meters)")
            plt.xlabel("Time (s)")
            plt.title("Pose Values")
        plt.show()

        dx_plot = np.array(dx[:, i]).reshape(1, np.array(dx).shape[0])

        for row in dx_plot:
            plt.plot(row)
            plt.ylabel("d(θ) (m/second)")
            plt.xlabel("Time (s)")
            plt.title("Pose Velocity")
        plt.show()

        ddx_plot = np.array(ddx[:, i]).reshape(1, np.array(dx).shape[0])

        for row in ddx_plot:
            plt.plot(row)
            plt.ylabel("d(d(θ)) (m/second^2)")
            plt.xlabel("Time (s)")
            plt.title("Pose Acceleration")
        plt.show()

def array_clean_print(sep=' ', vert='|', pad=10, precision=4):
    def prettyprint(a):
        s = "\n"
        if len(a.shape) == 1:
            a = [a]  # Make array appear 2D if it's 1D
        for row in a:
            s += vert + sep + sep.join([f"{x: ^ {pad}.{precision}g}" for x in row]) + vert + "\n"
        return s

    return prettyprint


np.set_string_function(array_clean_print(), repr=False)
np.set_string_function(array_clean_print(), repr=True)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from kinematics import jacobian as j
    from polynomial_trajectory import PolynomialGenerator

    pg = PolynomialGenerator()
    pgc = PolynomialGeneratorCart()

    x0 = [0.0756856730, -0.0000226960000, 0.533979, 0, 1.57079633, 0]
    dx0 = [0, 0, 0, 0, 0, 0]
    ddx0 = [0, 0, 0, 0, 0, 0]
    x_start = np.array([x0, dx0, ddx0])

    via_points = np.array([[0.0757261258, -0.12336681, 0.482877920, 1.57079633, 1.37079633, 2.67079633],
                           [0.07568567, -0.1744927, 0.359509, 1.57079633, 1.07079633, 1.77079633],
                           [0.07572613, -0.166367, 0.49851318, 0.79827042, 1.05037231, 1.93485616]])

    t_list = np.array([10.0, 20.0, 30.0])
    n_list = np.array([50, 50, 50])

    target = [0.07572613, -0.166367, 0.49851318, 0.79827042, 1.05037231, 1.93485616]
    x_final = [target]

    x, dx, ddx = pgc.generate_trajectory(x_start, via_points, 0, t_list, n_list)

    via_points = np.array([[0, np.pi / 4, -np.pi / 4, 0, 1.1, 0.2],
                           [0, np.pi / 2, -np.pi / 2, 0, 0.2, 0.5],
                           [0, 0.3, 0.2, 0.1, 1.1, 0.2]])

    q, dq, ddq = pg.generate_trajectory(x_start, via_points, 0, t_list, n_list)


    # x, dx, ddx = pgc.generate_p2p_trajectory(x_start, x_final, 0, 10, 100)
    # pgc.plot_xi(x, dx, ddx, 3)

    for i in range(len(x)):
        now = x[i]

        vel_control = np.linalg.inv(j.get_J(q[i])) @ dx[i]


        print(i, vel_control)

