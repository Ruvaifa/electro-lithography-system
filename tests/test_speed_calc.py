import unittest
import sys
import os

# Add parent directory to path to allow importing main and hmccontroller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import compute_synchronized_speeds

class TestSpeedCalculation(unittest.TestCase):
    def test_zero_displacement(self):
        self.assertEqual(compute_synchronized_speeds(0, 0, 50), (50, 50))

    def test_zero_speed(self):
        self.assertEqual(compute_synchronized_speeds(100, 200, 0), (0.0, 0.0))
        self.assertEqual(compute_synchronized_speeds(100, 200, -10), (0.0, 0.0))

    def test_proportional_speed(self):
        vx, vy = compute_synchronized_speeds(100, 200, 100)
        self.assertAlmostEqual(vx, 50.0)
        self.assertAlmostEqual(vy, 100.0)

    def test_bad_speed_avoidance_band_a(self):
        vx, vy = compute_synchronized_speeds(0, 100, 4.0)
        vy_steps = vy / 0.2
        self.assertTrue(vy_steps < 16 or vy_steps > 30, f"vy_steps {vy_steps} is in bad band A")

    def test_bad_speed_avoidance_band_b(self):
        vx, vy = compute_synchronized_speeds(0, 100, 30.0)
        vy_steps = vy / 0.2
        self.assertTrue(vy_steps < 123 or vy_steps > 244, f"vy_steps {vy_steps} is in bad band B")

if __name__ == '__main__':
    unittest.main()
