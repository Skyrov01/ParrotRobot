

from nicegui import Client, app, ui

import threading
import sys
import time

import requests
from threading import Lock, Thread

# These two are files where html, css and javascript are defined
from style import *
from script import *

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

AMPLITUDE_MAP = {
        "low": 15,
        "medium": 30,
        "high": 60,
    }


BASE_URL = "http://192.168.1.42:5000"

def generate_route_code(path_template: str, variables: dict):
    
    route = path_template
    for key, value in variables.items():
        route = route.replace(f"<{key}>", str(value))
    full_url = f"{BASE_URL}/{route}"
    code = f'requests.post("{full_url}")'
    return route, code

ui.add_css(style_parrot_control)

class MainPage:
    def __init__(self):
        pass
        self.dark_mode = False
        self.text_color = "text-white" if self.dark_mode else "text-gray-800"
        self.subtext_color = "text-gray-400" if self.dark_mode else "text-gray-600"
        self.accent_color = "text-purple-500"
        self.position_display = None

    def howto_section(self, title, command, description, topics):
        with ui.expansion(title, icon='rocket', value=False).classes("w-full"):
            ui.code(command)
            ui.label(description).classes(f"mb-2 {self.subtext_color}")
            ui.label(f"Topics: {topics}").classes(f"mb-2 {self.subtext_color}")

    def create_ui(self):
    
        with ui.column().style("padding: 0; gap: 0;").classes("w-full h-full"):
            with ui.card().classes("w-full"):
                with ui.element().classes("w-full").style("height: 35vh; width: 100%;"):
                    editor = ui.codemirror(default_code, language="python", theme="okaidia") \
                            .style("height: 100%; width: 100%;")

                # 20% Output
                with ui.element().classes("w-full h-1/5").style("height: 30%; width: 100%; "):
                    output_box = ui.textarea(label="Real-Time Output") \
                                .style("height: 100%; width: 100%; resize: none;")

                # 10% Buttons
                with ui.row().style("height: 10%; width: 100%; align-items: center; justify-content: left;"):
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

                    def cleanup():
                        try:
                            url = f"{BASE_URL}/servo/cleanup"
                            r = requests.post(url)
                            result = r.json()
                            ui.notify(result.get("status", "No response"))
                        except Exception as e:
                            ui.notify(f"Cleanup failed: {e}")

                    ui.button("Run Code", on_click=run, color="red").classes()
                    ui.button("Cleanup", on_click=cleanup, color="orange")

            with ui.card().classes("w-full mt-6"):
                with ui.row().classes("w-full gap-1 no-wrap"):
                    with ui.card().classes("w-1/2"):
                        ui.label("Robot Setup Parameters").classes("text-lg font-bold")

                        ui.label("Base URL").classes("mt-2")
                        base_url_input = ui.input(value=BASE_URL).classes("w-full").props('outlined')

                        def update_base_url():
                            global BASE_URL
                            BASE_URL = base_url_input.value
                            ui.notify(f"✅ BASE_URL updated to {BASE_URL}")

                        # Update when input loses focus or Enter is pressed
                        base_url_input.on('blur', lambda e: update_base_url())
                        base_url_input.on('keydown.enter', lambda e: update_base_url())
                        
                    with ui.card().classes("w-1/2"):
                        ui.label("Robot Description").classes("text-lg font-bold")
                
               # Container for the parrot image and controls
                ui.add_head_html(style_robot_controls_tailwind)
                ui.add_head_html(script_status_servo)
                # --- JavaScript for toggling popups on click ---
                ui.add_body_html(script_robot_controls_tailwind)

                def on_slider_change(target, e):
                    
                    self.position_display.value = f"Position: {e.value}°"
                    self.position_display.update()   
                    send_servo_command(target=target, position=e.value)
                    print(f"Slider changed to: {e.value}")  
                    
                
                with ui.element('div').classes('parrot-container w-full flex justify-center items-center relative'):
                    with ui.card().classes('w-full h-full relative'):
                        # Robot parrot interactive control
                        ui.label("Manual Control").classes("text-lg font-bold")
                        with ui.element('div').classes('relative mx-auto').style('width: 500px; height: 500px;'):
                            ui.image('static/images/parrot_robot.png').classes('absolute w-full h-full object-contain')

                            # === HEAD ===
                            ui.element('div').classes('hotspot').style('top: 90px; left: 250px;').props('id=head_circle')
                            with ui.element('div').classes('popup').style('top: -20px; left: 280px;').props('id=head_popup'):
                                ui.label('Head Rotate')
                                head_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("head_rotate", e)).props('label-always')
                                # ui.button('Send', on_click=lambda: self.send_servo('head_rotate', head_slider.value))

                            ui.element('div').classes('hotspot').style('top: 230px; left: 200px;').props('id=tilt_circle')
                            with ui.element('div').classes('popup').style('top: 280px; left: 150px;').props('id=tilt_popup'):
                                ui.label('Head Tilt')
                                head_slider = ui.slider(min=10, max=170, value=90, step=1,on_change=lambda e: on_slider_change("head_tilt", e)).props('label-always')

                            # === LEFT WING ===
                            ui.element('div').classes('hotspot').style('top: 220px; left: 90px;').props('id=left_circle')
                            with ui.element('div').classes('popup').style('top: 260px; left: -150px;').props('id=left_popup'):
                                ui.label('Left Wing')
                                left_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("left_wing", e)).props('label-always')
                                # ui.button('Send', on_click=lambda: self.send_servo('left_wing', left_slider.value))

                            # === RIGHT WING ===
                            ui.element('div').classes('hotspot').style('top: 220px; left: 380px;').props('id=right_circle')
                            with ui.element('div').classes('popup').style('top: 260px; left: 400px;').props('id=right_popup'):
                                ui.label('Right Wing')
                                right_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("right_wing", e)).props('label-always')
                                # ui.button('Send', on_click=lambda: self.send_servo('right_wing', right_slider.value))

                with ui.card().classes("w-full mt-6"):

                    # === Advanced Control Card ===
                    with ui.row().classes("w-full gap-1 no-wrap"):

                        with ui.card().classes('w-full md:w-1/2'):
                            ui.label("Advanced Control").classes("text-lg font-bold")
                            
                            with ui.row().classes("flex-nowrap items-end gap-2 overflow-hidden w-full"):
                                target_input = ui.select(
                                    ["head_rotate", "head_tilt", "left_wing", "right_wing"],
                                    value="head_rotate", label="Servo"
                                ).classes("w-full md:w-1/2")

                                position_input = ui.number("Position", value=90.0, format="%.1f").classes("w-1/6 md:w-1/2")

                                method_input = ui.select(["instant", "ease", "linear"], value="instant", label="Method").classes("w-1/5 md:w-1/2")

                                speed_input = ui.number("Speed", value=1.0, format="%.1f").classes("w-1/5 md:w-1/2")

                                ui.button("Send", on_click=lambda: send_servo_command()).classes("w-1/5 md:w-1/3 mt-2")

                                

                            # Live position display
                            self.position_display = ui.label("Position: 90.0°").classes("text-lg font-bold")
                            
                            output_area = ui.textarea(label="Response").classes("w-full h-32")
                            output_area.value = "Output will be displayed here"
                        
                        with ui.card().classes("w-full md:w-1/2 h-full"):
                            ui.label("Servo Status").classes("text-lg font-bold")
                            # Container for + pattern layout
                            with ui.element("div").classes("grid grid-cols-3 grid-rows-3 gap-2 w-full max-w-xs mx-auto place-items-center"):

                                def servo_status_circle(id, angle, insert=True):
                                    circle = ui.label(str(angle)).props(f"id={id}") \
                                        .classes(
                                            "rounded-full w-16 h-16 flex items-center justify-center text-white "
                                            "font-bold text-sm border-4 border-white shadow-md transition-all"
                                        ).style("background-color: gray;")
                                    if not insert:
                                        circle.classes("invisible")
                                    return circle

                                # Row 1: [empty, head tilt, empty]
                                servo_status_circle(None, "", insert=False)
                                servo_status_circle("servo_head_rotate", 90)
                                servo_status_circle(None, "", insert=False)

                                # Row 2: [left wing, empty, right wing]
                                servo_status_circle("servo_left_wing", 170)
                                servo_status_circle(None, "", insert=False)
                                servo_status_circle("servo_right_wing", 5)

                                # Row 3: [empty, head rotate, empty]
                                servo_status_circle(None, "", insert=False)
                                servo_status_circle("servo_head_tilt", 90)
                                servo_status_circle(None, "", insert=False)

                            ui.run_javascript("updateServoStatus('servo_head_tilt', 45, 'moving')")
                            ui.run_javascript("updateServoStatus('servo_right_wing', 0, 'ok')")
                      
                    
                    # position_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change)


                # -------------------------- Predefined Motions -----------------------------
                # Agree Card
                with ui.row().classes("w-full gap-1 no-wrap"):
                    with ui.card().classes("w-full md:w-1/2"):

                        ui.label("Agree Motion").classes("text-lg font-bold")

                        # --- Amplitude as toggle buttons ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Amplitude").classes("w-24")
                            amp = ui.toggle(["low", "medium", "high"], value="medium") \
                                    .props('color=red spread toggle-color=green').classes("w-1/2 flex-grow")

                        # --- Speed ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Speed").classes("w-24")
                            speed = ui.slider(min=0.1, max=5.0, value=1.0, step=0.1) \
                                    .props('label-always').classes("w-1/2 flex-grow")

                        # --- Repetitions ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Repetitions").classes("w-24")
                            reps = ui.slider(min=1, max=10, value=1, step=1) \
                                    .props('label-always').classes("w-1/2 flex-grow")

                        # --- Preview fields ---
                        route_label = ui.label().classes("mt-2 text-sm font-mono text-blue-600")
                        code_box = ui.input(label="Copy me!").classes("w-full h-8 font-mono")

                        def update_agree_preview():
                            route = f"/agree/amplitude/{amp.value}/speed/{speed.value}/repetitions/{int(reps.value)}"
                            full_url = f"{BASE_URL}{route}"
                            route_label.text = f"Route: {route}"
                            code_box.value = f'requests.post("{full_url}")'
                            return full_url

                        def execute_agree():
                            full_url = update_agree_preview()
                            try:
                                r = requests.post(full_url)
                                data = r.json()
                                ui.notify(f"✅ Command sent: {data}")
                            except Exception as e:
                                ui.notify(f"❌ Failed to send command: {e}")

                        # --- Bind updates ---
                        amp.on('update:model-value', lambda e: update_agree_preview())
                        speed.on('update:model-value', lambda e: update_agree_preview())
                        reps.on('update:model-value', lambda e: update_agree_preview())

                        ui.button("Send Command", on_click=execute_agree).classes("mt-2")

                        update_agree_preview()

                    with ui.card().classes("w-full md:w-1/2"):

                        ui.label("Disagree Motion").classes("text-lg font-bold")

                        # --- Amplitude as toggle buttons ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Amplitude").classes("w-24")
                            amp_dis = ui.toggle(["low", "medium", "high"], value="medium") \
                                        .props('color=red spread toggle-color=green').classes("w-1/2 flex-grow")

                        # --- Speed ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Speed").classes("w-24")
                            speed_dis = ui.slider(min=0.1, max=5.0, value=1.0, step=0.1) \
                                        .props('label-always').classes("w-1/2 flex-grow")

                        # --- Repetitions ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Repetitions").classes("w-24")
                            reps_dis = ui.slider(min=1, max=10, value=1, step=1) \
                                        .props('label-always').classes("w-1/2 flex-grow")

                        # --- Preview fields ---
                        route_label_dis = ui.label().classes("mt-2 text-sm font-mono text-blue-600")
                        code_box_dis = ui.input(label="Copy me!").classes("w-full h-8 font-mono")

                        def update_disagree_preview():
                            route = f"/disagree/amplitude/{amp_dis.value}/speed/{speed_dis.value}/repetitions/{int(reps_dis.value)}"
                            full_url = f"{BASE_URL}{route}"
                            route_label_dis.text = f"Route: {route}"
                            code_box_dis.value = f'requests.post("{full_url}")'
                            return full_url

                        def execute_disagree():
                            full_url = update_disagree_preview()
                            try:
                                r = requests.post(full_url)
                                data = r.json()
                                ui.notify(f"✅ Command sent: {data}")
                            except Exception as e:
                                ui.notify(f"❌ Failed to send command: {e}")

                        # --- Bind updates ---
                        amp_dis.on('update:model-value', lambda e: update_disagree_preview())
                        speed_dis.on('update:model-value', lambda e: update_disagree_preview())
                        reps_dis.on('update:model-value', lambda e: update_disagree_preview())

                        ui.button("Send Command", on_click=execute_disagree).classes("mt-2")

                        update_disagree_preview()

                with ui.row().classes("w-full gap-1 no-wrap"):
                    with ui.card().classes("w-full md:w-1/2"):

                        ui.label("Maybe Motion").classes("text-lg font-bold")

                        # --- Amplitude as toggle buttons ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Amplitude").classes("w-24")
                            amp_ind = ui.toggle(["low", "medium", "high"], value="medium") \
                                        .props('color=red spread toggle-color=green').classes("w-1/2 flex-grow")

                        # --- Speed ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Speed").classes("w-24")
                            speed_ind = ui.slider(min=0.1, max=5.0, value=1.0, step=0.1) \
                                        .props('label-always').classes("w-1/2 flex-grow")

                        # --- Repetitions ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Repetitions").classes("w-24")
                            reps_ind = ui.slider(min=1, max=10, value=1, step=1) \
                                        .props('label-always').classes("w-1/2 flex-grow")

                        # --- Preview fields ---
                        route_label_ind = ui.label().classes("mt-2 text-sm font-mono text-blue-600")
                        code_box_ind = ui.textarea(label="Python Code").classes("w-full h-8 font-mono")

                        def update_indifference_preview():
                            route = f"/maybe/amplitude/{amp_ind.value}/speed/{speed_ind.value}/repetitions/{int(reps_ind.value)}"
                            full_url = f"{BASE_URL}{route}"
                            route_label_ind.text = f"Route: {route}"
                            code_box_ind.value = f'requests.post("{full_url}")'
                            return full_url

                        def execute_indifference():
                            full_url = update_indifference_preview()
                            try:
                                r = requests.post(full_url)
                                data = r.json()
                                ui.notify(f"✅ Command sent: {data}")
                            except Exception as e:
                                ui.notify(f"❌ Failed to send command: {e}")

                        # --- Bind updates ---
                        amp_ind.on('update:model-value', lambda e: update_indifference_preview())
                        speed_ind.on('update:model-value', lambda e: update_indifference_preview())
                        reps_ind.on('update:model-value', lambda e: update_indifference_preview())

                        ui.button("Send Command", on_click=execute_indifference).classes("mt-2")

                        update_indifference_preview()

                    with ui.card().classes("w-full md:w-1/2"):

                        ui.label("Wave Motion").classes("text-lg font-bold")

                        # --- Wing selection ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Wing").classes("w-24")
                            wing_wave = ui.select(["left_wing", "right_wing"], value="left_wing") \
                                            .classes("w-1/2 flex-grow")

                        # --- Amplitude as toggle buttons ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Amplitude").classes("w-24")
                            amp_wave = ui.toggle(["low", "medium", "high"], value="medium") \
                                        .props('color=red spread toggle-color=green').classes("w-1/2 flex-grow")

                        # --- Speed ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Speed").classes("w-24")
                            speed_wave = ui.slider(min=0.1, max=5.0, value=1.0, step=0.1) \
                                            .props('label-always').classes("w-1/2 flex-grow")

                        # --- Repetitions ---
                        with ui.row().classes("items-center gap-4 w-full"):
                            ui.label("Repetitions").classes("w-24")
                            reps_wave = ui.slider(min=1, max=10, value=3, step=1) \
                                            .props('label-always').classes("w-1/2 flex-grow")

                        # --- Preview fields ---
                        route_label_wave = ui.label().classes("mt-2 text-sm font-mono text-blue-600")
                        code_box_wave = ui.textarea(label="Python Code").classes("w-full h-8 font-mono")

                        def update_wave_preview():
                            route = f"/wave/{wing_wave.value}/amplitude/{amp_wave.value}/speed/{speed_wave.value}/repetitions/{int(reps_wave.value)}"
                            full_url = f"{BASE_URL}{route}"
                            route_label_wave.text = f"Route: {route}"
                            code_box_wave.value = f'requests.post("{full_url}")'
                            return full_url

                        def execute_wave():
                            full_url = update_wave_preview()
                            try:
                                r = requests.post(full_url)
                                data = r.json()
                                ui.notify(f"✅ Command sent: {data}")
                            except Exception as e:
                                ui.notify(f"❌ Failed to send command: {e}")

                        # --- Bind updates ---
                        wing_wave.on('update:model-value', lambda e: update_wave_preview())
                        amp_wave.on('update:model-value', lambda e: update_wave_preview())
                        speed_wave.on('update:model-value', lambda e: update_wave_preview())
                        reps_wave.on('update:model-value', lambda e: update_wave_preview())

                        ui.button("Send Command", on_click=execute_wave).classes("mt-2")

                        update_wave_preview()


                def update_servo_ui(id, angle, status):
                    color = {
                        "ok": "green",
                        "moving": "orange",
                        "error": "red"
                    }.get(status, "gray")
                    ui.run_javascript(f'''
                        const el = document.getElementById('{id}');
                        if (el) {{
                            el.innerText = '{angle}';
                            el.style.backgroundColor = '{color}';
                        }}
                    ''')

                    # # Manual trigger button (in case you want to send explicitly)
                    # ui.button("Send Command", on_click=lambda: send_servo_command()).classes("mt-4")

                # Command sender function
                def send_servo_command(target=None, position=None):
                    host = BASE_URL
                    target = target if target is not None else target_input.value
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

        # --- Servo command function ---
    def send_servo(self, target, position):
        try:
            url = f"{BASE_URL}/servo/{target}/position/{position}/method/instant/speed/1.0"
            r = requests.post(url)
            ui.notify(f"{target} moved to {position}°")
        except Exception as e:
            ui.notify(f"Error: {e}")


@ui.page("/")
def main(client: Client):
    MainPage().create_ui()


ui.run(title="Papegaai", favicon='🦜', port=8080, reload=True)