from gpiozero import AngularServo
from time import sleep
import math
import threading


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
    "head_rotate": 5,
    "head_tilt": 170,
    "left_wing": LEFT_WING_DOWN,
    "right_wing": RIGHT_WING_DOWN
}

# Initialize servos with full angle range
servos = {
    name: AngularServo(pin, min_angle=0, max_angle=180,  min_pulse_width=0.0005, max_pulse_width=0.0025)
    for name, pin in servo_map.items()
}

servo_locks = {name: threading.Lock() for name in servo_map.keys()}

def init_servos():
    for name, angle in init_angles.items():
        if name in servos:
            servos[name].angle = angle
            last_angles[name] = angle
            print(f"[DRIVER][INFO] Initialized {name} to {angle}°")
        else:
            print(f"[DRIVER][WARN] Unknown servo: {name}")

# --- Movement Helpers ---
def move_instant(servo, current, target, speed):
    
    servo.angle = target
    # Estimate time needed based on angle change and speed
    wait_time = abs(target - current) * speed / 90  # 90°/s as baseline
    sleep(max(wait_time, 0.3))  # Always wait at least 0.3s



def move_linear(servo, current, target, speed=1.0):
    step = 1 if target > current else -1
    for a in range(current, target, step):
        servo.angle = a
        sleep(0.01 / speed)
    servo.angle = target


def move_ease_in_out(servo, current, target, speed=1.0):
    steps = 50
    for i in range(steps + 1):
        t = i / steps
        eased = 0.5 * (1 - math.cos(math.pi * t))
        a = current + (target - current) * eased
        servo.angle = a
        sleep(speed / steps)

# --- Core Move ---
def _do_move(name, angle, method="instant", speed=1.0):
    angle = int(float(angle))
    servo = servos[name]
    current = last_angles.get(name, 90)

    if method.lower() == "instant":
        move_instant(servo, current, angle, speed)
    elif method.lower() == "linear":
        move_linear(servo, current, angle, speed)
    else:
        move_ease_in_out(servo, current, angle, speed)

    last_angles[name] = angle

# --- Public API ---
def move_servo(name, angle, method="instant", speed=1.0, parallel=True):
    if name not in servos:
        print(f"[DRIVER][WARN] Unknown servo: {name}")
        return

    def task():
        print(f"[DRIVER][INFO] Trying to move {name} to {angle} ({method}, speed={speed})")
        with servo_locks[name]:
            print(f"[INFO] Moving {name} now...")
            _do_move(name, angle, method, speed)
        print(f"[DRIVER][INFO] Finished moving {name}")
        cleanup()

    print("Starting a servo task")
    if parallel:
        threading.Thread(target=task, daemon=True, name=f"servo-{name}").start()
    else:
        print("Not parralel")
        task()
    


# Move selected servos in parallel with individual angles, methods, and speeds.
# configs (dict): dictionary with servo names as keys and 
#                   a dict of {angle, method, speed} as values.
def move_selected_servos(configs):
    for name, cfg in configs.items():
        if name in servos:
            angle = cfg.get("angle", last_angles.get(name, 90))
            method = cfg.get("method", "instant")
            speed = cfg.get("speed", 1.0)
            move_servo(name, angle, method=method, speed=speed, parallel=True)
        else:
            print(f"[DRIVER][WARN] Unknown servo: {name}")

    print(f"[DRIVER][INFO] Configured move for servos: {list(configs.keys())}")


# Move all servos in parallel to the same angle.
def move_all_servos(angle, method="instant", speed=1.0):

    for name in servos.keys():
        move_servo(name, angle, method=method, speed=speed, parallel=True)

    print(f"[DRIVER][INFO] All servos commanded to {angle}° ({method}, speed={speed})")


def reset_servos(method="instant", speed=1.0):
    """
    Reset all servos to their default positions.
    Uses move_servo with parallel=True so reset happens simultaneously.
    """

    for name, angle in init_angles.items():
        if name in servos:
            move_servo(name, angle, method=method, speed=speed, parallel=True)
        else:
            print(f"[WARN] Unknown servo: {name}")

    print(f"[DRIVER][INFO] All servos reset with method={method}, speed={speed}")

def cleanup():
    for name, s in servos.items():
        try:
            if s.value is not None:
                
                s.detach()
                print(f"[DRIVER][CLEANUP] Detached servo: {name}")
        except Exception as e:
            print(f"[DRIVER][CLEANUP ERROR] Could not detach {name}: {e}")


if __name__ == "__main__":
    print("=== Servo Driver Standalone Test ===")

    # TEST 1: Move all servos together
    print("Moving all servos to 45° (linear)...")
    move_all_servos(45, method="linear", speed=1.0)
    sleep(2)

    print("Moving all servos back to 90° (ease)...")
    move_all_servos(90, method="ease", speed=1.0)
    sleep(2)

    # # TEST 2: Move only wings (different angles possible now)
    # print("Moving only wings...")
    # move_selected_servos({
    #     "left_wing": {"angle": 45, "method": "linear", "speed": 1.0},
    #     "right_wing": {"angle": 135, "method": "linear", "speed": 1.0},
    # })
    # sleep(2)

    # # TEST 3: Move only head servos
    # print("Moving only head servos...")
    # move_selected_servos({
    #     "head_rotate": {"angle": 120, "method": "ease", "speed": 1.0},
    #     "head_tilt": {"angle": 60, "method": "ease", "speed": 1.0},
    # })
    # sleep(2)

    # # TEST 4: Reset all servos
    # print("Resetting all servos...")
    # reset_servos(method="instant", speed=1.0)
    # sleep(2)

    cleanup()
    print("✅ Test complete, servos cleaned up.")