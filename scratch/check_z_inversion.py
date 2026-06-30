# scratch/check_z_inversion.py
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmccontroller import HmcControlCs

# Initialize controllers
z_hmc = HmcControlCs(axis='z')
x_hmc = HmcControlCs(axis='x')

# Assertions for logical Z inversion
assert z_hmc._logical_z_to_motor_z(100) == -100, f"Expected -100, got {z_hmc._logical_z_to_motor_z(100)}"
assert z_hmc._logical_z_to_motor_z(-50) == 50, f"Expected 50, got {z_hmc._logical_z_to_motor_z(-50)}"

# Assertions for X axis (no inversion)
assert x_hmc._logical_z_to_motor_z(100) == 100, f"Expected 100, got {x_hmc._logical_z_to_motor_z(100)}"

print("[SUCCESS] Z-axis inversion assertions passed.")
