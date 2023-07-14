import physics
import unittest


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


if __name__ == "__main__":
    unittest.main()
