import numpy as np
import scipy as sp

from util import make_cost_matrix, is_stabilizable
from discretize import discretize_ab


class LQR:
    """
    Linear Quadratic Regulator class
    """

    def __init__(self, A: np.ndarray, B: np.ndarray, Q_elems: np.ndarray, R_elems: np.ndarray, dt):
        """
        Constructs an LQR.
        :param A: continuous system matrix
        :param B: continuous input matrix
        :param Q_elems: The maximum error tolerance for each state.
        :param R_elems: The maximum desired control effort for each input.
        :param dt: The discretization timestep.
        """
        self.states = A.shape[0]
        self.inputs = B.shape[1]

        disc_A, disc_B = discretize_ab(A, B, dt)

        Q = make_cost_matrix(Q_elems)
        R = make_cost_matrix(R_elems)

        # solve for the optimal cost-to-go in the LQ problem.
        S = sp.linalg.solve_discrete_are(a=disc_A, b=disc_B, q=Q, r=R)
        self.K = np.linalg.solve(disc_B.T @ S @ disc_B + R, disc_B.T @ S @ disc_A)
        self.r = np.zeros((self.states, 1))
        self.u = np.zeros((self.states, 1))

    def reset(self):
        """
        Resets the controller by setting the R and U matrix to zero.
        :return: None
        """
        self.r = np.zeros((self.states, 1))
        self.u = np.zeros((self.states, 1))

    def calculate(self, x, r=None):
        """
        Calculate the next control step.
        :param x: the current state vector
        :param r: the reference vector
        :return:
        """
        if r is not None:
            self.r = r
        self.u = self.K @ (self.r - x)
        return self.u
