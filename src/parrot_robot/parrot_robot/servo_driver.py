# servo_driver.py
from RPi import GPIO
import time
import math

servo_map = {
    "head_rotate": 18,
    "head_tilt": 19,
    "left_wing": 17,
    "right_wing": 27
}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pwms = {}
last_angles = {}

for name, pin in servo_map.items():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)
    pwm.start(7.5)
    pwms[name] = pwm
    last_angles[name] = 90 if "head" in name else (170 if "left" in name else 5)

def angle_to_duty(angle):
    return 2.5 + (angle / 180.0) * 10

def move_servo(name, angle, method="Ease-In-Out", speed=1.0):
    angle = int(float(angle))
    current = last_angles[name]

    def set_duty(val):
        duty = angle_to_duty(val)
        pwms[name].ChangeDutyCycle(duty)

    if method == "Instant":
        set_duty(angle)
        time.sleep(0.1)
    elif method == "Linear":
        step = 1 if angle > current else -1
        for a in range(current, angle, step):
            set_duty(a)
            time.sleep(0.01)
        set_duty(angle)
    else:  # Ease-In-Out
        steps = 50
        for i in range(steps + 1):
            t = i / steps
            eased = 0.5 * (1 - math.cos(math.pi * t))
            a = current + (angle - current) * eased
            set_duty(a)
            time.sleep(speed / steps)
    pwms[name].ChangeDutyCycle(0)
    last_angles[name] = angle

def cleanup():
    for pwm in pwms.values():
        pwm.stop()
    GPIO.cleanup()
