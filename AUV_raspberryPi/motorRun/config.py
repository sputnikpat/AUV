# Config file for RPi GPIO configuration for ESC control

MAX_EMPTY_MSG_COUNT = 5
 
ESC_COMMANDS = ['calibrate', 'run', 'arm']

LOCALHOST = "127.0.0.1"

PWM_FREQUENCY = 50

USE_PIGPIO_MODULE = True

PWM_MAX_VAL = 1700  # change this if your ESC's max value is different or leave it be

PWM_MIN_VAL = 700  # change this if your ESC's min value is different or leave it be

DEMO_MODE_ON = True

