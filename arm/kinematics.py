import numpy as np
import scipy.linalg
import util
from util import SE3
import trimesh

# See also: arm_img.png in this same directory

# Constants
BRUSH_LENGTH = 0.115 + .1
HAND_LENGTH = .031
FOREARM_LENGTH = .149
UPPERARM_LENGTH = .148
ELBOW_LATERAL_OFFSET = 0.03205
BASE_TO_SHOULDER = 0.06525

# Lynch & Park M transforms and screw axes
REST_TRANSFORMS = [
    SE3.P([0, 0, BASE_TO_SHOULDER]),
    SE3.P([0, 0, UPPERARM_LENGTH]),
    SE3.P([ELBOW_LATERAL_OFFSET, 0, FOREARM_LENGTH]),
    SE3.P([0, 0, HAND_LENGTH]),
    SE3.P([0, 0, BRUSH_LENGTH]),
]
SCREW_AXES = np.array([
    [0, 0, 1, 0, 0, 0],
    [1, 0, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [1, 0, 0, 0, 0, 0],
])

# Forward Kinematics
def fk(theta, rest_Ts=REST_TRANSFORMS, screw_axes=SCREW_AXES):
    """
    Calculate the forward kinematics of a robot arm.
    Params:
        rest_Ts: A list of 4x4 numpy arrays representing the rest
            transformation matrices for each joint relative to
            the previous joint.
        screw_axes: A list of 6x1 numpy arrays representing the screw
            axes for each joint.
        theta: A list of joint angles.
    Returns:
        (T, Ts) where T is the overall transformation matrix and
        Ts is a list of the transformation matrices for each joint.
    """
    # Initialize transformation matrix
    T = SE3.Eye()
    Ts = []

    # Calculate the transformation matrix for each joint
    for i in range(len(theta)):
        # Calculate the rest transform for the i-th joint
        R = rest_Ts[i]

        # Calculate the screw transform for the i-th joint
        S = util.screw_transform(screw_axes[i])

        # Update the overall transformation matrix
        T = T @ scipy.linalg.expm(S * theta[i])
        Ts.append(T.copy())
        T = T @ R

    return T, Ts

# Jacobian calculation
def jacobian(theta, rest_Ts=REST_TRANSFORMS, screw_axes=SCREW_AXES):
    """
    Compute the Jacobian matrix of a robot arm using the Lynch and Park formula.
    Params:
        theta: A list of joint angles.
        rest_Ts: A list of 4x4 numpy arrays representing the rest
            transformation matrices for each joint relative to
            the previous joint.
        screw_axes: A list of 6x1 numpy arrays representing the screw
            axes for each joint.
    Returns:
        A 6xN numpy array representing the Jacobian matrix of the robot arm.
    """
    # Compute the transformation matrices for the current joint configuration
    T, Ts = fk(theta, rest_Ts, screw_axes)

    # Compute transformation matrices
    eTw = np.linalg.inv(T)
    eTis = [eTw @ wTi for wTi in Ts]

    # Initialize the Jacobian matrix
    J = np.zeros((6, len(theta)))

    # Compute the columns of the Jacobian matrix
    for i, (S_i, eTi) in enumerate(zip(screw_axes, eTis)):
        J[:, i] = util.Ad(eTi) @ screw_axes[i]

    return J


def ik(T,
       theta0=[0.1, 0.1, 1.0, 0.1, 0.1], # not zeros because of singularity, and prefer elbow_down
       rest_Ts=REST_TRANSFORMS,
       screw_axes=SCREW_AXES,
       max_iters=100,
       tol=1e-6,
       ignore_spin_axis=True,
       verbose=False,
       elbow_mode=None):
    """
    Compute the joint angles of a robot arm that achieve a desired end effector pose using inverse kinematics.
    Params:
        T: A 4x4 numpy array representing the desired end effector pose.
        theta0: A list of initial joint angles for the optimization.
        fk: A function that computes the forward kinematics of the robot arm.
        jacobian: A function that computes the Jacobian matrix of the robot arm.
        rest_Ts: A list of 4x4 numpy arrays representing the rest
            transformation matrices for each joint relative to
            the previous joint.
        screw_axes: A list of 6x1 numpy arrays representing the screw
            axes for each joint.
        max_iters: The maximum number of iterations for the optimization.
        tol: The tolerance for the optimization.
        ignore_spin_axis: If True, the spin of the brush will be ignored.
        verbose: If True, print the error at each iteration.
        elbow_mode: If None, the elbow can point in any direction.  If 'up' or 'down', elbow angle
            will be such that it's an elbow down-ish situation (how?).  If 'pos'/'neg', elbow angle
            will be forced to be positive/negative.
    Returns:
        A tuple (success, theta) where success is a boolean indicating whether the optimization succeeded,
        and theta is a list of joint angles that achieve the desired end effector pose.
    """
    # Initialize the joint angles
    theta = np.array(theta0)

    goalTw = np.linalg.inv(T)

    # Perform the optimization
    for _ in range(max_iters):
        # Canonicalize thetas
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        # Don't allow shoulder joint to spin beyond -90/90 degrees
        theta[0] = np.arctan(np.tan(theta[0]))
        if elbow_mode == 'pos':
            theta[2] = np.abs(theta[2])
        elif elbow_mode == 'neg':
            theta[2] = -np.abs(theta[2])
        elif elbow_mode == 'up':
            raise NotImplementedError
        elif elbow_mode == 'down':
            raise NotImplementedError

        # Compute the transformation matrices and Jacobian matrix for the current joint angles
        wTcur, _ = fk(theta, rest_Ts, screw_axes)
        J = jacobian(theta, rest_Ts, screw_axes)

        # Compute the error between the current end effector pose and the desired pose
        e = util.logm_to_vector(scipy.linalg.logm(goalTw @ wTcur))
        if verbose:
            print(e)
        if ignore_spin_axis:
            e[2] = 0

        # Check if the error is below the tolerance
        if np.linalg.norm(e) < tol:
            return True, theta

        if not ignore_spin_axis:
            # Compute the joint angle update using least squares sense, equivalent to pseudoinverse
            delta_theta, *_ = scipy.linalg.lstsq(J, e)
        else:
            # If we ignore the brush spin axis, then we are fully constrained
            # Remove row 2 from the Jacobian matrix and error
            Jalt = np.vstack((J[:2, :], J[3:, :]))
            e = np.hstack((e[:2], e[3:]))
            delta_theta = scipy.linalg.solve(Jalt, e)

        # Update the joint angles
        theta -= delta_theta

    # If we reached the maximum number of iterations without convergence, return failure
    return False, theta


######################## Visualization ########################
# Define Meshes
def DummyMeshes():
    create_link_mesh = lambda x: trimesh.creation.box(extents=[0.03, 0.03, x],
                                                    transform=SE3.P([0, 0, x / 2]))
    LINK_MESHES = [
        create_link_mesh(BASE_TO_SHOULDER),
        create_link_mesh(UPPERARM_LENGTH),
        create_link_mesh(FOREARM_LENGTH),
        create_link_mesh(HAND_LENGTH),
        create_link_mesh(BRUSH_LENGTH),
    ]
    LINK_MESHES[0].visual.face_colors = np.array([255, 0, 0, 255]) / 255.0
    LINK_MESHES[1].visual.face_colors = np.array([0, 255, 0, 255]) / 255.0
    LINK_MESHES[2].visual.face_colors = np.array([0, 0, 255, 255]) / 255.0
    LINK_MESHES[3].visual.face_colors = np.array([255, 255, 0, 255]) / 255.0
    LINK_MESHES[4].visual.face_colors = np.array([0, 255, 255, 255]) / 255.0
    return LINK_MESHES

def StlMeshes():
    def center_xy(mesh):
        mesh.apply_transform(SE3.P([-mesh.bounds[1][0]/2, -mesh.bounds[1][1]/2, 0]))
        return mesh

    L0 = trimesh.load_mesh('meshes/Base Joint.STL')
    L0.apply_scale(0.001)
    # L0.apply_transform(SE3.RotZ(np.pi/2) @ SE3.RotX(np.pi/2) @ SE3.P([-L0.bounds[1][0]/2, 0, -L0.bounds[1][-1] / 2]))
    L0.apply_transform(SE3.RotZ(np.pi/2) @ SE3.RotX(np.pi/2))
    L0 = center_xy(L0)

    L1a = trimesh.load_mesh('meshes/Wish Bone Coupling.STL')
    L1b = trimesh.load_mesh('meshes/Wish Bone Joint.STL')
    L1a.apply_scale(0.001)
    L1b.apply_scale(0.001)
    L1a.apply_transform(SE3.P([0, 0, 0.048]) @ SE3.RotX(np.pi))
    L1b.apply_transform(SE3.P([0, 0, 0.048]) @ SE3.RotZ(np.pi/2) @ SE3.RotX(np.pi/2))
    L1 = trimesh.util.concatenate(center_xy(L1a), center_xy(L1b))

    # L2 = DummyMeshes()[2] # TODO: Add this mesh
    L2 = trimesh.creation.box(extents=[0.01, 0.03, FOREARM_LENGTH], transform=SE3.P([ELBOW_LATERAL_OFFSET, 0, FOREARM_LENGTH / 2]))
    L2.visual.face_colors = np.array([200, 200, 200, 200])

    L3 = trimesh.load_mesh('meshes/Second Swivel.STL')
    L3.apply_scale(0.001)
    L3.apply_transform(SE3.RotX(np.pi/2))
    L3 = center_xy(L3)

    L4 = trimesh.load_mesh('meshes/2nd Servo Bracket.STL')
    L4.apply_scale(0.001)
    L4.apply_transform(SE3.P([0, 0, -.015]) @ SE3.RotX(np.pi/2))
    L4 = center_xy(L4)

    return [L0, L1, L2, L3, L4]

LINK_MESHES = StlMeshes()


# Visualization
def robot_meshes(joint_Ts, link_meshes=LINK_MESHES, color=None):
    if color is None:
        return [mesh.copy().apply_transform(T) for mesh, T in zip(link_meshes, joint_Ts)]
    else:
        meshes = [mesh.copy().apply_transform(T) for mesh, T in zip(link_meshes, joint_Ts)]
        for mesh in meshes:
            mesh.visual.face_colors = color
        return meshes

def create_scene(meshes):
    """
    Creates a scene containing meshes, with axes and the correct camera pose.
    """
    # Create a scene object to hold the robot arm meshes
    robot_scene = trimesh.Scene()

    # Add coordinate axes to the scene
    robot_scene.add_geometry(trimesh.creation.axis(axis_length=0.3, origin_size=0.01), node_name="axes")

    # Add the link meshes to the scene
    for i, mesh in enumerate(meshes):
        robot_scene.add_geometry(mesh, node_name=f'link_{i}')

    # Camera angle, view from side
    # robot_scene.apply_transform(SE3.RotX(-np.pi/2 + 0.2) @ SE3.RotZ(-0.2))
    robot_scene.apply_transform(SE3.RotY(0.1) @ SE3.RotX(0.2) @ SE3.RotZ(np.pi/2) @ SE3.RotY(np.pi/2))
    robot_scene.camera_transform = SE3.P([0, 0, 1.5])

    return robot_scene

def visualize_robot(joint_Ts, link_meshes=LINK_MESHES):
    """
    Creates a trimesh.Scene object representing a robot arm.
    Params:
        joint_Ts: A list of 4x4 numpy arrays representing the
            transformation matrices for each joint.
        link_meshes: A list of trimesh.Trimesh objects representing
            the meshes for each link.
    Returns:
        A trimesh.Scene object representing the robot arm.
    """
    return create_scene(robot_meshes(joint_Ts, link_meshes))
