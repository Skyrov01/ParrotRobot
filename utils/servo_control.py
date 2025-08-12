import lgpio
import tkinter as tk
import time
import threading
import math

servo_map = {
    "head_rotate": {"gpio": 18, "index": 0},
    "head_tilt": {"gpio": 19, "index": 1},
    "left_wing": {"gpio": 17, "index": 2},
    "right_wing": {"gpio": 27, "index": 3}
}

servo_pins = [servo_map[name]["gpio"] for name in servo_map]
servo_names = list(servo_map.keys())

FREQ = 50  # Hz
chip = lgpio.gpiochip_open(0)
SETTLE_TIME = 0.3

# Initialize GPIO outputs
for pin in servo_pins:
    lgpio.gpio_claim_output(chip, pin)

def angle_to_pulse(angle):
    return int(500 + (angle / 180.0) * 2000)



def send_angle(pin, angle):
    pulse = angle_to_pulse(angle)              # microseconds
    duty_cycle = pulse / 20000.0               # 20,000 us period at 50 Hz
    lgpio.tx_pwm(chip, pin, FREQ, duty_cycle)  # start/continue PWM

def stop_pwm(pin):
    # Fully stop PWM (no pulses). This is what actually "releases" the servo.
    lgpio.tx_pwm(chip, pin, 0, 0)

last_angles = [90, 90, 170, 5]

def move_instant(index, target_angle):
    pin = servo_pins[index]
    send_angle(pin, target_angle)
    time.sleep(SETTLE_TIME) 
    stop_pwm(pin)
    last_angles[index] = target_angle

def move_linear(index, target_angle, step_delay=0.01, step_size=1):
    pin = servo_pins[index]
    current_angle = last_angles[index]
    if target_angle == current_angle:
        return
    step = step_size if target_angle > current_angle else -step_size
    for angle in range(int(current_angle), int(target_angle), step):
        send_angle(pin, angle)
        time.sleep(step_delay)
    # Final target set + hold, then stop
    send_angle(pin, target_angle)
    time.sleep(SETTLE_TIME)
    stop_pwm(pin)
    last_angles[index] = target_angle

def move_ease_in_out(index, target_angle, duration=1.0, steps=50):
    pin = servo_pins[index]
    start_angle = last_angles[index]
    if start_angle == target_angle:
        return
    for i in range(steps + 1):
        t = i / steps
        eased_t = 0.5 * (1 - math.cos(math.pi * t))
        current_angle = start_angle + (target_angle - start_angle) * eased_t
        send_angle(pin, current_angle)
        time.sleep(duration / steps)
    time.sleep(SETTLE_TIME)
    stop_pwm(pin)
    last_angles[index] = target_angle

def move_servo(index, angle):
    method = selected_method.get()
    angle = int(float(angle))
    if method == "Instant":
        move_instant(index, angle)
    elif method == "Linear":
        move_linear(index, angle)
    elif method == "Ease-In-Out":
        try:
            duration = float(duration_entry.get())
            steps = int(steps_entry.get())
        except ValueError:
            duration = 1.0
            steps = 50
        move_ease_in_out(index, angle, duration=duration, steps=steps)

def threaded_move(index, angle):
    thread = threading.Thread(target=move_servo, args=(index, angle))
    thread.start()

root = tk.Tk()
root.title("Parrot Servo Controller")

selected_method = tk.StringVar(value="Ease-In-Out")
method_menu = tk.OptionMenu(root, selected_method, "Instant", "Linear", "Ease-In-Out")
method_menu.pack(pady=5)

param_frame = tk.Frame(root)
param_frame.pack(pady=10)

tk.Label(param_frame, text="Ease-In-Out Duration (s):", font=("Arial", 12)).pack(side="left", padx=5)
duration_entry = tk.Entry(param_frame, width=5, font=("Arial", 12))
duration_entry.insert(0, "1.0")
duration_entry.pack(side="left", padx=5)

tk.Label(param_frame, text="Steps:", font=("Arial", 12)).pack(side="left", padx=5)
steps_entry = tk.Entry(param_frame, width=5, font=("Arial", 12))
steps_entry.insert(0, "50")
steps_entry.pack(side="left", padx=5)

sliders = []

def slider_click(event, slider, index):
    widget = event.widget
    if widget["orient"] == "horizontal":
        new_val = widget["from"] + ((event.x / widget.winfo_width()) * (widget["to"] - widget["from"]))
    else:
        new_val = widget["from"] + ((event.y / widget.winfo_height()) * (widget["to"] - widget["from"]))
    slider.set(new_val)
    threaded_move(index, new_val)

for i, name in enumerate(servo_names):
    frame = tk.Frame(root)
    frame.pack(pady=10)

    label_text = f"{name.replace('_', ' ').title()} (GPIO {servo_pins[i]})"
    tk.Label(frame, text=label_text, font=("Arial", 14)).pack(anchor="w")

    slider = tk.Scale(frame,
                      from_=5,
                      to=175,
                      orient=tk.HORIZONTAL,
                      length=400,
                      sliderlength=30,
                      width=20,
                      font=("Arial", 12),
                      command=lambda val, idx=i: threaded_move(idx, val))
    slider.set(last_angles[i])
    slider.pack()
    slider.bind("<Button-1>", lambda event, s=slider, idx=i: slider_click(event, s, idx))

    control_frame = tk.Frame(frame)
    control_frame.pack()

    angle_entry = tk.Entry(control_frame, width=5, font=("Arial", 12))
    angle_entry.insert(0, str(last_angles[i]))
    angle_entry.pack(side="left", padx=5)

    def make_set_button(index=i, entry=angle_entry):
        return lambda: threaded_move(index, entry.get())

    set_button = tk.Button(control_frame, text="Set", font=("Arial", 12),
                           command=make_set_button())
    set_button.pack(side="left", padx=5)

    sliders.append(slider)

def on_closing():
    for pin in servo_pins:
        lgpio.tx_pwm(chip, pin, 0, 0)
    lgpio.gpiochip_close(chip)
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

for i, angle in enumerate(last_angles):
    move_instant(i, angle)
    sliders[i].set(angle)

root.mainloop()