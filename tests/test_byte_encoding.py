import unittest
import sys
import os

# Add parent directory to path to allow importing main and hmccontroller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmccontroller import HmcControlCs

class TestByteEncoding(unittest.TestCase):
    def setUp(self):
        self.hmc = HmcControlCs(axis='z')

    def test_msb_csb_lsb(self):
        # Value: 5000 -> 0x1388 -> [0, 19, 136]
        self.assertEqual(self.hmc.msb_csb_lsb(5000), [0, 19, 136])
        # Value: 0 -> [0, 0, 0]
        self.assertEqual(self.hmc.msb_csb_lsb(0), [0, 0, 0])
        # Negative value (should use absolute value)
        self.assertEqual(self.hmc.msb_csb_lsb(-5000), [0, 19, 136])

    def test_byte3_byte2_byte1_byte0(self):
        # Value: 1000 -> 0x3E8 -> [0, 0, 3, 232]
        self.assertEqual(self.hmc.byte3_byte2_byte1_byte0(1000), [0, 0, 3, 232])
        # Value: 16777216 -> 0x1000000 -> [1, 0, 0, 0]
        self.assertEqual(self.hmc.byte3_byte2_byte1_byte0(16777216), [1, 0, 0, 0])

    def test_msb_lsb(self):
        # Value: 1000 -> 0x3E8 -> [3, 232]
        self.assertEqual(self.hmc.msb_lsb(1000), [3, 232])

if __name__ == '__main__':
    unittest.main()
