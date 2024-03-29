import RPi.GPIO as GPIO
import argparse, time

# setup motors
IN1 = 16
IN2 = 20
ENA = 21

IN3 = 13
IN4 = 19
ENB = 26
# setup pompa
IN5 = 23
# Numiration of exits
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(IN5, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
# PWM frequency - speed of robot
p1 = GPIO.PWM(ENA, 50)
p2 = GPIO.PWM(ENB, 50)


def MotorAB_Direction_Forward(x, y):
    try:
        global IN1, IN2, IN3, IN4, p1, p2
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        p1.start(x)
        p2.start(y)
    except:
        print(x, y)


def MotorAB_Direction_Backward(x, y):
    try:
        global IN1, IN2, IN3, IN4, p1, p2
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        p1.start(x)
        p2.start(y)
    except:
        print(x, y)


def MotorAB_Direction_Right(x, y):
    try:
        global IN1, IN2, IN3, IN4, p1, p2
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        p1.start(x)
        p2.start(y)
        # time.sleep(0.5)
    except:
        print(x, y)


def MotorAB_Direction_Left(x, y):
    try:
        global IN1, IN2, IN3, IN4, p1, p2
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        p1.start(x)
        p2.start(y)
        # time.sleep(5)
    except:
        print(x, y)


def MotorAB_Brake():
    global IN1, IN2, IN3, IN4, p1, p2
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.HIGH)
    p1.start(0)
    p2.start(0)


def MotorAB_Direction_Suck():
    global IN5
    GPIO.output(IN5, GPIO.HIGH)
    return True


def MotorAB_Direction_Unsuck():
    global IN5
    GPIO.output(IN5, GPIO.LOW)
    return True

def MotorAB_Direction_Pickup():
    import Adafruit_PCA9685
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    try:
        #open fingers
        pwm.set_pwm(4, 0, 500)
        time.sleep(2)
        #left-right
        pwm.set_pwm(0, 0, 350)
        time.sleep(2)
        # 1 - 250 back or 350-400 forward
        pwm.set_pwm(1, 0, 390)
        time.sleep(2)
        # 2 - cubit 550 - forward
        #pwm.set_pwm(2, 0, 550)
        #time.sleep(2)
        # 3 - wrist down or up. 450 - forward
        pwm.set_pwm(3, 0, 450)
        time.sleep(2)
        # 4 - fingers. 500 - open, 370 full close
        pwm.set_pwm(4, 0, 380)
        time.sleep(2)
        ### and go back
        pwm.set_pwm(1, 0, 250)
        time.sleep(2)
    except KeyboardInterrupt:
        print('Interrupted')
    else:
        return True


def MotorAB_Direction_Stack():
    import Adafruit_PCA9685
    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    try:
        #left-right
        pwm.set_pwm(0, 0, 350)
        time.sleep(2)
        # 1 - 250 back or 350-400 forward
        pwm.set_pwm(1, 0, 380)
        time.sleep(2)
        # 2 - cubit
        #pwm.set_pwm(2, 0, 550)
        #time.sleep(2)
        # 3 - wrist down or up. 450 - forward
        pwm.set_pwm(3, 0, 450)
        time.sleep(2)
        # 4 - fingers. 500 - open, 370 full close
        pwm.set_pwm(4, 0, 500)
        time.sleep(2)
        ### and go back
        pwm.set_pwm(1, 0, 250)
        time.sleep(2)
    except KeyboardInterrupt:
        print('Interrupted')
    else:
        return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(dest="direct", nargs='?')
    args = parser.parse_args()

    dir = args.direct
    start_time = time.time()
    print("Start time is %s" % str(start_time))
    flag = True
    while flag:
        try:
            func = globals()['MotorAB_Direction_' + dir]
            func(30, 30)
            time.sleep(1)
        except KeyboardInterrupt:
            flag = False
            MotorAB_Brake()
            finish_time = time.time()
            print("Finish time is %s" % str(finish_time))
            print("Moved for %s sec" % str(finish_time - start_time))




