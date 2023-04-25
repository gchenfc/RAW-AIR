import numpy as np
import scipy.linalg


class SE3:
    def Eye():
        """
        Returns a 4x4 identity matrix.
        """
        return np.eye(4)

    def RotX(theta):
        """
        Returns a 4x4 homogeneous transformation matrix for rotation about the x-axis by angle theta.
        """
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[1, 0, 0, 0],
                        [0, c, -s, 0],
                        [0, s, c, 0],
                        [0, 0, 0, 1]])

    def RotY(theta):
        """
        Returns a 4x4 homogeneous transformation matrix for rotation about the y-axis by angle theta.
        """
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[c, 0, s, 0],
                        [0, 1, 0, 0],
                        [-s, 0, c, 0],
                        [0, 0, 0, 1]])

    def RotZ(theta):
        """
        Returns a 4x4 homogeneous transformation matrix for rotation about the z-axis by angle theta.
        """
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[c, -s, 0, 0],
                        [s, c, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    def P(t):
        """
        Returns a 4x4 translation matrix for translation by vector t.
        """
        T = np.eye(4)
        T[:3, 3] = np.array(t)
        return T


def screw_transform(screw_params):
    """
    Returns the screw transform matrix for a joint given its screw parameters.
    Assumes the following screw parameter convention:
    - The first three values are the screw axis (unit vector).
    - The last three values are the point on the screw axis (position vector).
    """
    w = screw_params[:3]  # screw axis
    p = screw_params[3:]  # point on screw axis
    theta = np.linalg.norm(w)  # angle of rotation about screw axis

    if theta < 1e-6:
        # If theta is very small, use a simplified formula
        V = np.eye(3) + skew(w) + 0.5 * skew(w) @ skew(w)
        S = np.vstack((np.hstack((V, np.zeros((3, 1)))), np.array([0, 0, 0, 0])))
    else:
        # Otherwise, use the full formula
        # A = np.sin(theta) / theta
        # B = (1 - np.cos(theta)) / (theta**2)
        # C = (1 - A) / (theta**2)
        wx = skew(w)
        # V = np.eye(3) + A * wx + B * wx @ wx
        # p_hat = skew(p)
        # T = np.eye(3) - C * wx + (1/2) * (wx @ wx)
        # G = T @ p_hat @ wx + p.reshape(-1, 1) @ w.reshape(1, -1)
        # print(V.shape, G.shape)
        S = np.vstack((np.hstack((wx, p[:, None])), np.array([0, 0, 0, 0])))

    return S

def logm_to_vector(M):
    """
    Convert a 4x4 matrix logarithm to a 6-vector in the tangent space of SE(3).
    Params:
        M: A 4x4 numpy array representing the matrix logarithm of a homogeneous transformation matrix.
    Returns:
        A 6x1 numpy array representing the 6-vector in the tangent space of SE(3).
    """
    R = M[:3, :3]
    p = M[:3, 3]
    omega = skew_inv(R)
    return np.concatenate((omega, p))

def skew(v):
    """
    Returns the skew-symmetric matrix of a vector.
    """
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def skew_inv(S):
    """
    Convert a 3x3 skew-symmetric matrix to a 3x1 vector.
    Params:
        S: A 3x3 numpy array representing a skew-symmetric matrix.
    Returns:
        A 3x1 numpy array representing the vector that corresponds to S.
    """
    v = np.array([S[2, 1], S[0, 2], S[1, 0]])
    return v

def Ad(T):
    """
    Calculate the Adjoint matrix of a homogeneous transformation matrix.
    Params:
        T: A 4x4 numpy array representing the homogeneous transformation matrix.
    Returns:
        A 6x6 numpy array representing the Adjoint matrix.
    """
    R = T[:3, :3]
    p = T[:3, 3]
    zeros = np.zeros((3, 3))
    return np.block([[R, zeros], [skew(p) @ R, R]])

def counts2deg(counts: int):
    """
    Convert encoder counts to degrees.
    """
    return (counts / 1023 * 300 - 150)
def deg2counts(angle: float):
    """
    Convert degrees to encoder counts.
    """
    return int((angle + 150) / 300 * 1023)
