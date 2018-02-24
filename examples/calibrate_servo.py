# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)


TOTAL_TICKS = 4096
middle_pulse_period = 1.5
min_pulse_period    = 2.0
max_pulse_period    = 1.0
pwm_period          = 20.0

freq = int(1000/ pwm_period)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(freq)

def calculate_ticks(angle=90):
    return int(204.8 / 180 * angle + 204.8)

def map_from_angle_to_tick(value, from_min=0, from_max=180, to_min=servro_min, to_max=servo_max):
    tick = (to_max - to_min) / float(from_max - from_min) * (value - from_min) + to_min
    print("tick:" + str(tick))
    return int(tick)

def test_run():
    pwm.set_pwm(15, 0, calculate_ticks())
    text = input("Continute?")
    print('Moving servo on channel 0, press Ctrl-C to quit...')
    while True:
        # Move servo on channel O between extremes.
        pwm.set_pwm(15, 0, calculate_ticks(0))
        time.sleep(1)
        pwm.set_pwm(15, 0, calculate_ticks(180))
        time.sleep(1)

print(map_from_angle_to_tick(90))
