from gpiozero import AngularServo
from time import sleep
import math

RIGHT_WING_UP = 5
RIGHT_WING_DOWN = 170
LEFT_WING_UP = 170
LEFT_WING_DOWN = 5

# Map servo names to GPIO pins
servo_map = {
    "head_rotate": 18,
    "head_tilt": 19,
    "left_wing": 17,
    "right_wing": 27
}

# Last known angles
last_angles = {
    "head_rotate": 90,
    "head_tilt": 90,
    "left_wing": LEFT_WING_UP,
    "right_wing": RIGHT_WING_UP
}

# Initial angles
init_angles = {
    "head_rotate": 90,
    "head_tilt": 90,
    "left_wing": LEFT_WING_UP,
    "right_wing": RIGHT_WING_UP
}

test_angles = {
    "head_rotate": 90,
    "head_tilt": 90,
    "left_wing": LEFT_WING_DOWN,
    "right_wing": RIGHT_WING_DOWN
}

# Initialize servos with full angle range
servos = {
    name: AngularServo(pin, min_angle=0, max_angle=180,  min_pulse_width=0.0005, max_pulse_width=0.0025)
    for name, pin in servo_map.items()
}


def init_servos():
    for name, angle in init_angles.items():
        if name in servos:
            servos[name].angle = angle
            last_angles[name] = angle
            print(f"[INFO] Initialized {name} to {angle}°")
        else:
            print(f"[WARN] Unknown servo: {name}")


def move_servo(name, angle, method="instant", speed=1.0):
    angle = int(float(angle))
    if name not in servos:
        print(f"[WARN] Unknown servo: {name}")
        return

    servo_obj = servos[name]
    current = last_angles.get(name, 90)

    if method == "instant":
        servo_obj.angle = angle
        # Estimate time needed based on angle change and speed
        wait_time = abs(angle - current) * speed / 90  # 90°/s as baseline
        print(f"[INFO] Instant move {name} to {angle}° (current: {current}°), estimated wait: {wait_time:.2f}s")
        sleep(max(wait_time, 0.3))  # Always wait at least 0.3s

    elif method == "linear":
        step = 1 if angle > current else -1
        for a in range(current, angle, step):
            servo_obj.angle = a
            sleep(0.01)
        servo_obj.angle = angle
    elif method == "ease":  # Ease-In-Out
        steps = 50
        for i in range(steps + 1):
            t = i / steps
            eased = 0.5 * (1 - math.cos(math.pi * t))
            a = current + (angle - current) * eased
            servo_obj.angle = a
            sleep(speed / steps)

    last_angles[name] = angle

def cleanup():
    for s in servos.values():
        s.detach()