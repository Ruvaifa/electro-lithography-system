# config.py
# ponytail: flat config of constants, simple and importable directly.

BAUD_RATE = 19200
RESOLUTION_UM_PER_STEP = 0.2  # 0.2 um/step (Resolution_A/B/C)
MAX_TRAVEL_UM = 50000         # 50 mm (50000 um)
MAX_STEPS = 16777216          # maximum steps (2^24)
MIN_SPEED_STEPS = 10
MAX_SPEED_STEPS = 50000.0
HOMING_SPEED_STEPS = 5000
SAFE_MOVE_SPEED_STEPS = 5000

# Protocol byte codes
CMD_DEVICE = 100
ACK_DEVICE = 200
CMD_OK = 10
CMD_MOVE = 19
CMD_SPEED = 34
CMD_STOP = 104
ACK_STOP = 105
CMD_READ = 163
ACK_READ = 164
CMD_ACCELERATION = 20
CMD_DECELERATION = 30
DIR_PLUS = 125
DIR_MINUS = 175
MOVE_COMPLETED = 170

# Limit switch response codes
LIMIT_CODES = {
    40: 'x_far',
    41: 'x_home',
    42: 'y_far',
    43: 'y_home',
    44: 'z_far',
    45: 'z_home'
}

# Firmware bad speed bands (empirically measured in steps/s)
BAD_SPEED_BANDS = [
    (16, 30),
    (123, 244)
]

# Serial timeouts
ACK_TIMEOUT_S = 2.0
AXIS_READ_TIMEOUT_S = 2.0
SERIAL_READ_TIMEOUT_S = 0.1
SERIAL_WRITE_TIMEOUT_S = 0.1
