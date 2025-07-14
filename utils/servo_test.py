from gpiozero import AngularServo
from time import sleep

# Configure the servo on GPIO 18
# Adjust min/max pulse width if motion is limited or jittery
servo = AngularServo(
    18,                  # GPIO pin
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,  # 0.5ms
    max_pulse_width=0.0025   # 2.5ms
)

try:
    for angle in [0, 90, 180, 90]:
        print(f"Setting angle to {angle}°")
        servo.angle = angle
        sleep(1)
finally:
    print("Detaching servo")
    servo.detach()
