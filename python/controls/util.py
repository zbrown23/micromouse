"""Random utility functions for controls stuff."""

import numpy as np
import scipy as sp


def make_cost_matrix(elems: np.ndarray) -> np.ndarray:
    """
    Creates a cost matrix from a given vector so that we can use it in LQR.
    The cost matrix is constructed using Bryson's rule, where each diagonal
    entry is the inverse square of the corresponding element in the input vector.

    For a Q matrix, the input elements represent the maximum allowed deviations
    of the states from their reference vector. For an R matrix, the input elements
    represent the maximum allowed deviations of the control inputs from no actuation.

    :param elems: Input vector with maximum deviation values
    :return: The state excursion (Q) or control effort cost (R) matrix
    """
    elems = np.array(elems, dtype=float)
    # Calculate the inverse square of each element, setting inf elements to 0
    diagonal_values = np.where(elems == float("inf"), 0.0, 1.0 / elems ** 2)

    # Create a diagonal matrix from the processed values
    cost_matrix = np.diag(diagonal_values)

    return cost_matrix


def is_stabilizable(A: np.ndarray, B: np.ndarray) -> bool:
    """
    returns whether the system defined by A and B is stabilizable.
    (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A,
    if any, have absolute values less than one.
    :param A: system matrix
    :param B: input matrix
    :return: whether (A, B) is stabilizable
    """
    # Compute eigenvalues and eigenvectors of A
    eigenvalues = sp.linalg.eig(A)

    for i, eigenvalue in enumerate(eigenvalues):
        # Check if the eigenvalue is uncontrollable by checking the controllability matrix of (A-λI,B)
        # If the rank of this matrix is less than the number of states, n, then the eigenvalue is uncontrollable.
        controllable = np.linalg.matrix_rank(np.column_stack([A - eigenvalue * np.eye(A.shape[0]), B])) == A.shape[0]

        # If uncontrollable and eigenvalue has a non-negative real part, system is not stabilizable
        if not controllable and eigenvalue.real >= 0:
            return False

    return True
