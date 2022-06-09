import numpy as np


def average_quaternion(q_list):
    """
    Computes the average orientation from a list of quaternions.
    Based on: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
    Args:
        q_list(list[np.ndarray]): List of normalized quaternion vectors to average. q = [x, y, z, w]

    Returns:
        np.ndarray: average quaternion
    """
    # Index checking
    n_quat = len(q_list)

    # Construct Q matrix
    Q = np.zeros([4, n_quat])
    for i in range(n_quat):
        Q[:, i] = np.asarray(q_list[i])

    # Compute M matrix
    M = np.dot(Q, Q.T)

    # Compute mean orientation as the eigenvector corresponding to the largest eigenvalue
    w, v = np.linalg.eig(M)
    q_avg = v[:, np.argmax(w)]

    return q_avg
