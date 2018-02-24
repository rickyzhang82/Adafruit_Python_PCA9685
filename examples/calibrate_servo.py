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
SERVO_MIN = 120  # Min pulse length out of 4096
SERVO_MAX = 535  # Max pulse length out of 4096
ANGLE_MIN = 0
ANGLE_MAX = 180
TOTAL_TICKS = 4096
FREQ = 50
SERVO_CHANNEL=15
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(FREQ)

def map_from_angle_to_tick(value, from_min=ANGLE_MIN, from_max=ANGLE_MAX, to_min=SERVO_MIN, to_max=SERVO_MAX):
    tick = (to_max - to_min) / float(from_max - from_min) * (value - from_min) + to_min
    print("tick:" + str(tick))
    return int(tick)

def bound_tick(value, to_min=SERVO_MIN, to_max=SERVO_MAX):
    return min(max(SERVO_MIN, value), SERVO_MAX)

def calibrate_by_angle():
    print('Moving servo on channel %d, press Ctrl-C to quit...'%(SERVO_CHANNEL))
    while True:
        angle=int(input("Input angle [%d, %d]:"%(ANGLE_MIN, ANGLE_MAX)))
        tick=bound_tick(map_from_angle_to_tick(angle))
        print("bounded tick: " + str(tick))
        pwm.set_pwm(SERVO_CHANNEL, 0, tick)
        time.sleep(1)

def calibrate_by_tick():
    print('Moving servo on channel %d, press Ctrl-C to quit...'%(SERVO_CHANNEL))
    while True:
        tick=int(input("Input tick [%d, %d]:"%(TICK_MIN, TICK_MAX)))
        tick=bound_tick(tick)
        print("bounded tick: " + str(tick))
        pwm.set_pwm(SERVO_CHANNEL, 0, tick)
        time.sleep(1)

# first test calibrate by tick to find the max and min tick
# last verify it by angle
