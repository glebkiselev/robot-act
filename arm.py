from __future__ import division
import time
import argparse
import Adafruit_PCA9685

#
pwm = Adafruit_PCA9685.PCA9685()
#
servo_min = 300
servo_max = 500  # max imp 4096


#

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000
    # 1,000,000 us per second
    pulse_length //= 60
    # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096
    # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)


# freq 60 gc
pwm.set_pwm_freq(60)
# print('Moving servo on channel 0, press Ctrl-C to quit...')
# while True:
# move servo
# pwm.set_pwm(0, 0, servo_min)
# time.sleep(1)
# pwm.set_pwm(0, 0, servo_max)
# time.sleep(1)
# pwm.set_pwm(1, 0, 100)
# time.sleep(1)
# pwm.set_pwm(1, 0, servo_max)
# time.sleep(1)
if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(dest="pwm_num", nargs='?')
    parser.add_argument(dest="place", nargs='?')
    args = parser.parse_args()

    pwm_num = int(args.pwm_num)
    place = int(args.place)
    try:
        pwm.set_pwm(pwm_num, 0, place)
        time.sleep(1)
    except KeyboardInterrupt:
        print('Interrupted')



