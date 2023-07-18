import physics
import unittest
import numpy as np


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(10, 1000), 98100)
        self.assertNotEqual(physics.calculate_buoyancy(10, 1000), 0)
        self.assertRaises(ValueError, physics.calculate_buoyancy, -10, -10)

    def test_will_it_float(self):
        self.assertEqual(physics.will_it_float(10, 10), True)
        self.assertNotEqual(physics.will_it_float(10, 10), False)
        self.assertRaises(ValueError, physics.will_it_float, -10, -10)

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(100), 981000 + 101325)
        self.assertNotEqual(physics.calculate_pressure(100), 9.81)
        self.assertRaises(ValueError, physics.calculate_pressure, -10)

    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(10, 10), 1)
        self.assertNotEqual(physics.calculate_acceleration(10, 10), 2)
        self.assertRaises(ValueError, physics.calculate_acceleration, 0, 0)
        self.assertRaises(ValueError, physics.calculate_acceleration, 5, -10)

    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(10, 5), 2)
        self.assertNotEqual(physics.calculate_angular_acceleration(10, 10), 2)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, 0, 0)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, -5, 0)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, -5, -5)
        self.assertRaises(ValueError, physics.calculate_angular_acceleration, 0, -5)

    def test_calculate_torque(self):
        self.assertEqual(physics.calculate_torque(10, 360, 5), 50)
        self.assertEqual(physics.calculate_torque(10, -360, 5), 50)
        self.assertNotEqual(physics.calculate_torque(10, 360, 5), 15)
        self.assertRaises(ValueError, physics.calculate_torque, -10, 10, -10)
        self.assertRaises(ValueError, physics.calculate_torque, 0, 10, 10)
        self.assertRaises(ValueError, physics.calculate_torque, 10, 10, -10)
        self.assertRaises(ValueError, physics.calculate_torque, 10, 10, 0)
        self.assertRaises(ValueError, physics.calculate_torque, -10, 10, 10)

    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(5, 10), 500)
        self.assertNotEqual(physics.calculate_moment_of_inertia(5, 10), 0)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, 10, 0)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, 0, 10)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, -10, 10)
        self.assertRaises(ValueError, physics.calculate_moment_of_inertia, 10, -10)

    def test_calculate_auv_acceleration(self):
        self.assertEqual(physics.calculate_auv_acceleration(100, 0), 1)
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 101, 0)
        self.assertRaises(ValueError, physics.calculate_auv_acceleration, 1, 45)
        self.assertRaises(
            ValueError, physics.calculate_auv_acceleration, -10, -10, -10, -10, -10
        )

    def test_calculate_auv_angular_acceleration(self):
        self.assertEqual(physics.calculate_auv_angular_acceleration(100, 0), 50)
        self.assertRaises(
            ValueError, physics.calculate_auv_angular_acceleration, 101, 0
        )
        self.assertRaises(ValueError, physics.calculate_auv_angular_acceleration, 1, 45)
        self.assertRaises(
            ValueError, physics.calculate_auv_angular_acceleration, -10, -10, -10, -10
        )

    def test_calculate_auv2_acceleration(self):
        self.assertTrue(
            np.allclose(
                physics.calculate_auv2_acceleration(
                    np.array([100, 100, 100, 100]), np.pi / 2, 0
                ),
                np.array([0, 0]),
            )
        )
        self.assertRaises(
            ValueError, physics.calculate_auv2_acceleration, np.array([110]), 0, 0
        )

    def test_calculate_auv2_angular_acceleration(self):
        self.assertEqual(
            physics.calculate_auv2_angular_acceleration(
                np.array([40, 20, 30, 10]), np.pi / 2, 5, 10
            ),
            4,
        )
        self.assertNotEqual(
            physics.calculate_auv2_angular_acceleration(
                np.array([40, 20, 30, 10]), np.pi / 2, 5, 10
            ),
            2,
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv2_angular_acceleration,
            np.array([110]),
            20,
            20,
            20,
            20,
        )
        self.assertRaises(
            ValueError,
            physics.calculate_auv2_angular_acceleration,
            np.array([10, -10, 10, -10]),
            20,
            20,
            -20,
            -20,
        )

    def test_simulate_auv2_motion(self):
        (
            t_test,
            x_test,
            y_test,
            theta_test,
            v_test,
            omega_test,
            a_test,
        ) = physics.simulate_auv2_motion(
            np.array([1, 0, 1, 0]), 0.5, 1.5, 1.8, dt=0.5, t_final=1.5
        )
        self.assertEqual(t_test[0], 0)
        self.assertEqual(t_test[1], 0.5)
        self.assertEqual(t_test[2], 1.0)
        self.assertEqual(np.all(x_test), 0)
        self.assertEqual(np.all(y_test), 0)
        self.assertEqual(theta_test[0], 0)
        self.assertEqual(theta_test[1], 0.010896699061615622)
        self.assertEqual(theta_test[2], 0.03269009718484686)
        self.assertEqual(np.all(v_test), 0)
        self.assertEqual(np.all(a_test), 0)
        self.assertEqual(omega_test[0], 0)
        self.assertEqual(omega_test[1], 0.021793398123231243)
        self.assertRaises(
            ValueError,
            physics.simulate_auv2_motion,
            np.array([1, 0]),
            0.5,
            1.5,
            1.8,
            dt=0.5,
            t_final=1.5,
        )
        self.assertRaises(
            ValueError,
            physics.simulate_auv2_motion,
            np.array([1, 0, 1, 0]),
            0.5,
            -1.5,
            1.8,
            dt=0.5,
            t_final=1.5,
        )
        self.assertRaises(
            ValueError,
            physics.simulate_auv2_motion,
            np.array([1, 0, 1, 0]),
            0.5,
            1.5,
            -1.8,
            dt=0.5,
            t_final=1.5,
        )
        self.assertRaises(
            ValueError,
            physics.simulate_auv2_motion,
            np.array([1, 0, 1, 0]),
            0.5,
            1.5,
            1.8,
            dt=-0.5,
            t_final=1.5,
        )
        self.assertRaises(
            ValueError,
            physics.simulate_auv2_motion,
            np.array([1, 0, 1, 0]),
            0.5,
            1.5,
            1.8,
            dt=0.5,
            t_final=-1.5,
        )


if __name__ == "__main__":
    unittest.main()
