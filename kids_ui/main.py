

from nicegui import ui
import threading
import sys
import time

import requests
from threading import Lock, Thread

class RealTimeOutput:
    def __init__(self, output_box):
        self.output_box = output_box

    def write(self, text):
        if text.strip():
            self.output_box.value += text + "\n"
            self.output_box.update()

    def flush(self):
        pass

# Default Python code
default_code = """import time
for i in range(5):
    print(f"Line {i}")
    time.sleep(0.5)
"""

# Fullscreen layout
position_display = None

with ui.column().style("height: 100vh; padding: 0; gap: 0;").classes("w-full h-full"):

    # 70% Editor
    with ui.element().style("height: 70%; width: 100%;"):
        editor = ui.codemirror(default_code, language="python", theme="okaidia") \
                  .style("height: 100%; width: 100%;")

    # 20% Output
    with ui.element().style("height: 20%; width: 100%;"):
        output_box = ui.textarea(label="Real-Time Output") \
                      .style("height: 100%; width: 100%; resize: none;")

    # 10% Buttons
    with ui.row().style("height: 10%; width: 100%; align-items: center; justify-content: left; padding: 10px;"):
        def run_user_code():
            output_box.value = ""
            output = RealTimeOutput(output_box)
            old_stdout = sys.stdout
            sys.stdout = output
            print("Running user code...")
            try:
                exec(editor.value, {}, {})
            except Exception as e:
                print(f"Error: {e}")
            finally:
                sys.stdout = old_stdout

        def run():
            threading.Thread(target=run_user_code).start()

        ui.button("Run Code", on_click=run).classes("w-32")

    with ui.column().classes("w-1/2"):
        ui.label("Servo Control").classes("text-lg font-bold")
        # Inputs
        with ui.row().classes("gap-4"):
            target_input = ui.select(["head_rotate", "head_tilt", "left_wing", "right_wing"], value="head_rotate", label="Servo")   
            position_input = ui.number("Position", value=90.0, format="%.1f")
            method_input = ui.select(["instant", "ease", "linear"], value="instant", label="Method")   
            speed_input = ui.number("Speed", value=1.0, format="%.1f")
            ui.button("Send Command", on_click=lambda: send_servo_command()).classes("w-32")

        output_area = ui.textarea(label="Response").classes("w-full h-32")

        # Live position display
        position_display = ui.label("Position: 90.0°").classes("text-lg font-bold")

        # Slider for position

        # Optional: update the servo live as slider moves (you can disable if you prefer button-based control)
        def on_slider_change(e):
            global position_display
            position_display.value = f"Position: {e.value}°"
            position_display.update()   
            send_servo_command(position=e.value)
            print(f"Slider changed to: {e.value}")  
            

        position_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change)



        # Manual trigger button (in case you want to send explicitly)
        ui.button("Send Command", on_click=lambda: send_servo_command()).classes("mt-4")

        # Command sender function
        def send_servo_command(position=None):
            host = "http://192.168.1.42:5000"
            target = target_input.value
            position = float(position) if position is not None else position_input.value
            speed = speed_input.value
            method = method_input.value

            url = f"{host}/servo/{target}/position/{position}/method/{method}/speed/{speed}"

            try:
                r = requests.post(url)
                data = r.json()
                output_area.value = f"✅ Success:\n{data}"
                ui.notify("Command sent!")
            except Exception as e:
                output_area.value = f"❌ Error:\n{e}"
                ui.notify("Failed to send command")






ui.run()
    