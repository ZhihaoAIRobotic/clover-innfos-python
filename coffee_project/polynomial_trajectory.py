import numpy as np
from math import comb, factorial

class PolynomialGenerator():
    """
    PolynomialGenerator generates a polynomial trajectory given a set of initial and final conditions.
    """

    def __init__(self):
        pass

    def get_constraint_submatrix(self, t, n):
        """
        Get the time constraint submatrix for a given time t

        Args:
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
            Array of n points of the polynomial (Joint Values)
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
            Array of n points of the polynomial's dth derivative (Joint Velocities or Accelerations)
        """

        ts = np.linspace(from_t, to_t, n)
        poly = 0

        if d >= len(coefs):
            print("Not enough coefficients to generate derivative of order {}".format(d))
            return np.zeros(n)

        for i, coef in enumerate(coefs[d:], d):
            poly += coef * factorial(d) * comb(i, d) * ts**(i-d)

        return poly

    def generate_p2p_trajectory(self, q_init, q_final, t_f, t_0=0.0, n=100):

        """
        Generates a len(q_init)th DOF trajectory given initial and final conditions

        Args:
            q_init (list): Initial conditions (Type: Array)
            q_final (list): Final conditions (Type: Array)
            t_f (float): Final time
            t_0 (float): Initial time
            n (int): Number of points to generate
        """

        q = q_init[0]
        dq = q_init[1]
        ddq = q_init[2]

        if len(q_final) == 1:
            qf = q_final[0]
            q_desired = [qf]

        elif len(q_final) == 2:
            qf = q_final[0]
            dqf = q_final[1]
            q_desired = [qf, dqf]
        elif len(q_final) == 3:
            qf = q_final[0]
            dqf = q_final[1]
            ddqf = q_final[2]
            q_desired = [qf, dqf, ddqf]

        qlist = np.zeros((len(q), n))
        dqlist = np.zeros((len(dq), n))
        ddqlist = np.zeros((len(ddq), n))

        for i in range(len(q)-1):
            q_in = [q[i], dq[i], ddq[i]]

            if len(q_desired) == 1:
                q_d = [q_desired[0][i]]
            elif len(q_desired) == 2:
                q_d = [q_desired[0][i], q_desired[1][i]]
            elif len(q_desired) == 3:
                q_d = [q_desired[0][i], q_desired[1][i], q_desired[2][i]]

            qlist[i] = self.polynomial_from_coefs(self.generate_coefficients(q_in, q_d, t_f, t_0), t_0, t_f, n)
            dqlist[i] = self.dpolynomial_from_coefs(1, self.generate_coefficients(q_in, q_d, t_f, t_0), t_0, t_f, n)
            ddqlist[i] = self.dpolynomial_from_coefs(2, self.generate_coefficients(q_in, q_d, t_f, t_0), t_0, t_f, n)

        qlist = qlist.T
        dqlist = dqlist.T
        ddqlist = ddqlist.T

        return qlist, dqlist, ddqlist

    def generate_trajectory(self, q_init, via_points, from_t, t_list, n_list):
        """
        Generates a len(q_init)th DOF trajectory given initial and final conditions

        Args:
            q_init (list): Initial conditions (Type: Array)
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

        qall_list = np.zeros((total, len(q_init[0])))
        dqall_list = np.zeros((total, len(q_init[0])))
        ddqall_list = np.zeros((total, len(q_init[0])))
        length = 0

        for i in range(len(via_points)):
            if i == 0:
                via0 = via_points[0].reshape(1, 6)
                q_temp, dq_temp, ddq_temp = self.generate_p2p_trajectory(q_init, via0, t_list[0], from_t, n_list[0])
                qall_list[:len(q_temp)] = q_temp
                dqall_list[:len(q_temp)] = dq_temp
                ddqall_list[:len(q_temp)] = ddq_temp
            else:
                length = length + len(q_temp)
                via = via_points[i].reshape(1, 6)
                con_temp = np.array([q_temp[-1], dq_temp[-1], ddq_temp[-1]])
                q_temp, dq_temp, ddq_temp = self.generate_p2p_trajectory(con_temp, via, t_list[i], t_list[i-1], n_list[i])
                qall_list[length: (length + len(q_temp))] = q_temp
                dqall_list[length: (length + len(q_temp))] = dq_temp
                ddqall_list[length: (length + len(q_temp))] = ddq_temp

        return qall_list, dqall_list, ddqall_list
    def plot_all(self, q, dq, ddq):
        """
        Plots the joint values, velocities and accelerations with respect to time

        Args:
            q (np.array): Joint values
            dq (np.array): Joint velocities
            ddq (np.array): Joint accelerations

        Returns:
            A plot of the joint values, velocities and accelerations with respect to time
        """
        q_plot = np.array(q).reshape(6, np.array(q).shape[0])

        for row in q_plot:
            plt.plot(row)
            plt.ylabel("θ (rads)")
            plt.xlabel("Time (s)")
            plt.title("Joint Values")
        plt.show()

        dq_plot = np.array(dq).reshape(6, np.array(dq).shape[0])

        for row in dq_plot:
            plt.plot(row)
            plt.ylabel("d(θ) (rads/second)")
            plt.xlabel("Time (s)")
            plt.title("Joint Velocity")
        plt.show()

        ddq_plot = np.array(ddq).reshape(6, np.array(dq).shape[0])

        for row in ddq_plot:
            plt.plot(row)
            plt.ylabel("d(d(θ)) (rads/second^2)")
            plt.xlabel("Time (s)")
            plt.title("Joint Acceleration")
        plt.show()

    def plot_qi(self, q, dq, ddq, i):
        """
        Plots joint value[i], velocity[i] and acceleration[i] with respect to time

        Args:
            q (np.array): Joint values
            dq (np.array): Joint velocities
            ddq (np.array): Joint accelerations

        Returns:
            A plot of the joint values, velocities and accelerations with respect to time
        """
        q_plot = np.array(q[:, i]).reshape(1, np.array(q).shape[0])

        for row in q_plot:
            plt.plot(row)
            plt.ylabel("θ (rads)")
            plt.xlabel("Time (s)")
            plt.title("Joint Values")
        plt.show()

        dq_plot = np.array(dq[:, i]).reshape(1, np.array(dq).shape[0])

        for row in dq_plot:
            plt.plot(row)
            plt.ylabel("d(θ) (rads/second)")
            plt.xlabel("Time (s)")
            plt.title("Joint Velocity")
        plt.show()

        ddq_plot = np.array(ddq[:, i]).reshape(1, np.array(dq).shape[0])

        for row in ddq_plot:
            plt.plot(row)
            plt.ylabel("d(d(θ)) (rads/second^2)")
            plt.xlabel("Time (s)")
            plt.title("Joint Acceleration")
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

    pg = PolynomialGenerator()

    q0 = [0, 0, 0, 0, 0, 0]
    dq0 = [0, 0, 0, 0, 0, 0]
    ddq0 = [0, 0, 0, 0, 0, 0]
    q_start = np.array([q0, dq0, ddq0])
    via_points = np.array([[0, np.pi/4, -np.pi/4, 0, 1.1, 0.2],
                           [0, np.pi/2, -np.pi/2, 0, 0.2, 0.5],
                           [0, 0.3, 0.2, 0.1, 1.1, 0.2],
                           [0, 0, 0, 0, 0, 0]])
    t_list = np.array([10.0, 15.0, 20.0, 21.0])
    n_list = np.array([100, 50, 50, 10])

    q, dq, ddq = pg.generate_trajectory(q_start, via_points, 0, t_list, n_list)

    pg.plot_all(q, dq, ddq)


