import unittest
import hello
import numpy as np
import math


class TestHello(unittest.TestCase):
    def test_hello(self):
        self.assertEqual(hello.hello(), "Hello, world!")

    def test_sin(self):
        self.assertEqual(hello.sin(0), 0)
        self.assertEqual(hello.sin(1), 0.8414709848078965)
        self.assertEqual(round(hello.sin(math.pi), 15), 0)

    def test_cos(self):
        self.assertEqual(hello.cos(0), 1)
        self.assertEqual(hello.cos(1), 0.5403023058681398)
        self.assertEqual(round(hello.cos(np.pi), 15), -1)

    def test_tan(self):
        self.assertEqual(hello.tan(0), 0)
        self.assertEqual(hello.tan(1), 1.5574077246549023)
        self.assertEqual(round(hello.tan(np.pi), 15), 0)

    def test_cot(self):
        self.assertEqual(hello.cot(0), float("inf"))
        self.assertEqual(hello.cot(1), 0.6420926159343306)
        self.assertEqual(hello.cot(2), -0.45765755436028577)

    def test_add(self):
        self.assertEqual(hello.add(1, 2), 3)
        self.assertEqual(hello.add(3, 4), 7)
        self.assertNotEqual(hello.add(1, 1), 3)

    def test_sub(self):
        self.assertEqual(hello.sub(1, 2), -1)
        self.assertEqual(hello.sub(3, 4), -1)
        self.assertNotEqual(hello.sub(1, 1), 0.1)

    def test_mul(self):
        self.assertEqual(hello.mul(1, 2), 2)
        self.assertEqual(hello.mul(3, 4), 12)
        self.assertNotEqual(hello.mul(1, 1), 2)

    def test_div(self):
        self.assertEqual(hello.div(1, 2), 0.5)
        self.assertEqual(hello.div(3, 4), 0.75)
        self.assertNotEqual(hello.div(1, 1), 0)

    def test_sqrt(self):
        self.assertEqual(hello.sqrt(9), 3)
        self.assertEqual(hello.sqrt(4), 2)
        self.assertNotEqual(hello.sqrt(1), 0)

    def test_power(self):
        self.assertEqual(hello.power(1, 2), 1)
        self.assertEqual(hello.power(3, 4), 81)
        self.assertNotEqual(hello.power(5, 1), 6)

    def test_log(self):
        self.assertEqual(hello.log(1), 0)
        self.assertEqual(hello.log(2), 0.6931471805599453)
        self.assertNotEqual(hello.log(0), 0)

    def test_exp(self):
        self.assertEqual(hello.exp(0), 1)
        self.assertEqual(hello.exp(1), 2.718281828459045)
        self.assertEqual(hello.exp(2), 7.389056098930650)


if __name__ == "__main__":
    unittest.main()
