from gpiozero import AngularServo
from time import sleep
import math



# Map servo names to GPIO pins
servo_map = {
    "head_rotate": 18,
    "head_tilt": 19,
    "left_wing": 17,
    "right_wing": 27
}

# Initialize servos with full angle range
servos = {
    name: AngularServo(pin, min_angle=0, max_angle=180,  min_pulse_width=0.0005, max_pulse_width=0.0025)
    for name, pin in servo_map.items()
}

# Last known angles
last_angles = {
    "head_rotate": 90,
    "head_tilt": 90,
    "left_wing": 170,
    "right_wing": 5
}



def move_servo(name, angle, method="Instant", speed=1.0):
    angle = int(float(angle))
    if name not in servos:
        print(f"[WARN] Unknown servo: {name}")
        return

    servo = servos[name]
    current = last_angles.get(name, 90)

    if method == "Instant":
        servo.angle = angle
        sleep(0.1)
    elif method == "Linear":
        step = 1 if angle > current else -1
        for a in range(current, angle, step):
            servo.angle = a
            sleep(0.01)
        servo.angle = angle
    else:  # Ease-In-Out
        steps = 50
        for i in range(steps + 1):
            t = i / steps
            eased = 0.5 * (1 - math.cos(math.pi * t))
            a = current + (angle - current) * eased
            servo.angle = a
            sleep(speed / steps)

    last_angles[name] = angle

def cleanup():
    for s in servos.values():
        s.detach()