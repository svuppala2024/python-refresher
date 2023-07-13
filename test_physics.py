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


if __name__ == "__main__":
    unittest.main()
