import unittest
import numpy as np
import util
from util import SE3
import kinematics
from kinematics import (BASE_TO_SHOULDER, UPPERARM_LENGTH, FOREARM_LENGTH, HAND_LENGTH,
                        BRUSH_LENGTH, ELBOW_LATERAL_OFFSET)
from kinematics import REST_TRANSFORMS, SCREW_AXES
import scipy.linalg

a, b, c, d, e = BASE_TO_SHOULDER, UPPERARM_LENGTH, FOREARM_LENGTH, HAND_LENGTH, BRUSH_LENGTH
off = ELBOW_LATERAL_OFFSET

class TestKinematics(unittest.TestCase):

    def test_all_zero_joint_angles(self):
        T_actual, _ = kinematics.fk([0, 0, 0, 0, 0], REST_TRANSFORMS, SCREW_AXES)
        T_expected = SE3.P([off, 0, a + b + c + d + e])
        np.testing.assert_allclose(T_actual, T_expected)

    def test_rot_shoulder_spin(self):
        T_actual, _ = kinematics.fk([np.pi / 2, 0, 0, 0, 0], REST_TRANSFORMS, SCREW_AXES)
        T_expected = SE3.P([0, off, a + b + c + d + e]) @ SE3.RotZ(np.pi / 2)
        np.testing.assert_allclose(T_actual, T_expected, atol=1e-8)

    def test_rot_shoulder_down(self):
        T_actual, _ = kinematics.fk([0, -np.pi / 2, 0, 0, 0], REST_TRANSFORMS, SCREW_AXES)
        T_expected = SE3.P([off, b + c + d + e, a]) @ SE3.RotX(-np.pi / 2)
        np.testing.assert_allclose(T_actual, T_expected, atol=1e-8)

    def test_all_pi_over_2_joint_angles(self):
        T_actual, _ = kinematics.fk([np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2, np.pi / 2],
                                    REST_TRANSFORMS, SCREW_AXES)
        T_expected = SE3.P([b, off + e, a - c - d]) @ SE3.RotX(-np.pi / 2)
        np.testing.assert_allclose(T_actual, T_expected, atol=1e-8)

    def test_skew_and_inv(self):
        np.random.seed(8675309)
        for _ in range(3):
            v = np.random.randn(6)
            np.testing.assert_allclose(util.logm_to_vector(util.screw_transform(v)), v)

    def test_jacobian_numerical(self):
        np.random.seed(8675309)

        for _ in range(3):
            # Define some random joint angles
            theta = np.random.randn(5)

            # Calculate the Jacobian matrix analytically
            J_analytical = kinematics.jacobian(theta)

            # Define a small perturbation for each joint angle
            eps = 1e-6

            # Calculate the Jacobian matrix numerically using central differences
            J_numerical = np.zeros_like(J_analytical)
            for i in range(len(theta)):
                theta_perturbed = theta.copy()
                theta_perturbed[i] += eps
                T_perturbed, _ = kinematics.fk(theta_perturbed)
                T, _ = kinematics.fk(theta)
                delta_T = util.logm_to_vector(scipy.linalg.logm(np.linalg.inv(T) @ T_perturbed))
                J_numerical[:, i] = delta_T / eps

            # Compare the analytical and numerical Jacobian matrices
            np.testing.assert_allclose(J_analytical, J_numerical, rtol=1e-4, atol=1e-6)

    def test_ik_consistency(self):
        for _ in range(3):
            np.random.seed(8675309)

            # Create a robot arm with random joint angles
            theta_exp = np.random.uniform(-np.pi / 2, np.pi / 2, size=5)

            # Compute the end effector pose for the current joint angles
            T_exp, _ = kinematics.fk(theta_exp)

            # Use the IK function to compute the joint angles that achieve the current end effector pose
            success, theta_act = kinematics.ik(T_exp, ignore_spin_axis=False) # avoid RR issues

            # Check that the IK succeeded
            self.assertTrue(success)

            # Check that the IK joint angles are close to the GT joint angles
            np.testing.assert_allclose(theta_exp, theta_act, rtol=1e-6, atol=1e-6)

            # Check that the end effector pose computed using the IK joint angles is close to the desired pose
            T_act, _ = kinematics.fk(theta_act)
            np.testing.assert_allclose(T_exp, T_act, rtol=1e-6, atol=1e-6)

    def test_ik_useful(self):
        pose_paint_dip = SE3.P([0, -0.2, 0]) @ SE3.RotX(-np.pi)

        success, ik_result = kinematics.ik(pose_paint_dip, ignore_spin_axis=True, elbow_mode='pos')
        assert success, f'IK failed: {ik_result}'

        thetas_expected = [1.609439e-01, 1.180903e-01, 1.437257e+00, 3.98000e-08, 1.586245e+00]
        np.testing.assert_allclose(ik_result, thetas_expected, rtol=1e-6, atol=1e-6)

if __name__ == '__main__':
    unittest.main()



if __name__ == '__main__':
    unittest.main()
