from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


class ServoCalibrator:
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685()
    # Configure min and max servo pulse lengths
    DEFAULT_TICK_MIN = 120  # Min pulse length out of 4096
    DEFAULT_TICK_MAX = 535  # Max pulse length out of 4096
    ANGLE_MIN = 0
    ANGLE_MAX = 180
    TOTAL_TICKS = 4096
    FREQ = 50
    DEFAULT_SERVO_CHANNEL = 15
    SERVO_CHANNEL_MIN = 0
    SERVO_CHANNEL_MAX = 15

    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(FREQ)

    def __init__(self):
        self.tick_min = ServoCalibrator.DEFAULT_TICK_MIN
        self.tick_max = ServoCalibrator.DEFAULT_TICK_MAX
        self.servo_channel = ServoCalibrator.DEFAULT_SERVO_CHANNEL
        pass

    @staticmethod
    def map_from_angle_to_tick(value,
                               from_min=ANGLE_MIN, from_max=ANGLE_MAX,
                               to_min=DEFAULT_TICK_MIN, to_max=DEFAULT_TICK_MAX):
        tick = (to_max - to_min) / float(from_max - from_min) * (value - from_min) + to_min
        print("tick:" + str(tick))
        return int(tick)

    @staticmethod
    def bound_tick(value, to_min=DEFAULT_TICK_MIN, to_max=DEFAULT_TICK_MAX):
        return min(max(ServoCalibrator.DEFAULT_TICK_MIN, value), ServoCalibrator.DEFAULT_TICK_MAX)

    @staticmethod
    def calibrate_by_angle(servo_channel=DEFAULT_SERVO_CHANNEL, tick_min=DEFAULT_TICK_MIN, tick_max=DEFAULT_TICK_MAX):
        print('Moving servo on channel %d, enter -1 to quit...' % servo_channel)
        while True:
            angle = int(input("Input angle [%d, %d]:" % (ServoCalibrator.ANGLE_MIN, ServoCalibrator.ANGLE_MAX)))
            if -1 == angle:
                break

            tick = ServoCalibrator.bound_tick(
                ServoCalibrator.map_from_angle_to_tick(angle, to_min=tick_min, to_max=tick_max))
            print("bounded tick: " + str(tick))
            ServoCalibrator.pwm.set_pwm(servo_channel, 0, tick)
            time.sleep(1)

    @staticmethod
    def calibrate_by_tick(servo_channel=DEFAULT_SERVO_CHANNEL):
        print('Moving servo on channel %d, enter -1 to quit...' % servo_channel)
        while True:
            tick = int(input("Input tick [%d, %d]:" %
                             (ServoCalibrator.DEFAULT_TICK_MIN, ServoCalibrator.DEFAULT_TICK_MAX)))
            if -1 == tick:
                break
            tick = ServoCalibrator.bound_tick(tick)
            print("bounded tick: " + str(tick))
            ServoCalibrator.pwm.set_pwm(servo_channel, 0, tick)
            time.sleep(1)

    def calibrate(self):
        servo_channel_str = input("Enter servo channel [%d, %d]:" %
                          (ServoCalibrator.SERVO_CHANNEL_MIN, ServoCalibrator.SERVO_CHANNEL_MAX))

        if len(servo_channel_str) != 0:
            self.servo_channel = min(max(int(servo_channel_str), ServoCalibrator.SERVO_CHANNEL_MIN),
                                     ServoCalibrator.SERVO_CHANNEL_MAX)
        print("Set servo channel %d" % self.servo_channel)
        # first test calibrate by tick to find the max and min tick
        ServoCalibrator.calibrate_by_tick(self.servo_channel)

        # last verify it by angle
        print("\n*****************************************************")
        print("Verify tick by angle")
        print("*****************************************************\n")
        self.tick_max = int(input("Enter servo MAX ticks:"))
        self.tick_min = int(input("Enter servo MIN ticks:"))
        if self.tick_min >= self.tick_max or self.tick_min < 0 or self.tick_min > ServoCalibrator.TOTAL_TICKS or \
            self.tick_max < 0 or self.tick_max > ServoCalibrator.TOTAL_TICKS:
            print("Incorrect ticks!")
            return
        ServoCalibrator.calibrate_by_angle(servo_channel=self.servo_channel,
                                           tick_min=self.tick_min, tick_max=self.tick_max)


if __name__ == '__main__':
    calibrator = ServoCalibrator()
    calibrator.calibrate()