import numpy as np
from scipy import linalg as la


def discretize_a(A, dt):
    """
    Discretizes the continuous time A matrix.
    :param A: continuous time A matrix
    :param dt: the discretization timestep
    :return: the discrete time A matrix.
    """
    return la.expm(A * dt)


def discretize_ab(A, B, dt):
    """
    Discretizes the continuous time A and B matrices by exploiting the solution to an autonomous
    first order differential equation plus the input dynamics. We first create an augmented matrix M,
    then exponentiate e to the power of this augmented matrix times dt. This gives us a single timestep solution.
    The augmented matrix is important as we cannot discretize the input dynamics represented by the B matrix separately.
    :param A: the continuous time system matrix
    :param B: the continuous time system matrix
    :param dt: the discretization timestep
    :return: the discrete time A and B matrices.
    """
    num_states = A.shape[0]
    num_inputs = B.shape[1]

    # formulate the augmented matrix
    # looks like:
    # [A  B]
    # [0  0]
    m = la.expm(np.block([[A, B], [np.zeros((num_inputs, num_states)), np.zeros((num_inputs, num_inputs))]]) * dt)
    return m[:num_states, :num_states], m[:num_states, num_states:]
