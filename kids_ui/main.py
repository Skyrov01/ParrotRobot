

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

                    ui.button("Run Code", on_click=run, color="red").classes()

            with ui.card().classes("w-full mt-6"):
                with ui.row().classes("w-full gap-1 no-wrap"):
                    with ui.card().classes("w-1/2"):
                        ui.label("Robot Setup Parameters").classes("text-lg font-bold")
                        
                    with ui.card().classes("w-1/2"):
                        ui.label("Robot Description").classes("text-lg font-bold")
                
               # Container for the parrot image and controls
                ui.add_head_html(style_robot_controls_tailwind)
                # --- JavaScript for toggling popups on click ---
                ui.add_body_html(script_robot_controls_tailwind)

                def on_slider_change(e):
         
                    self.position_display.value = f"Position: {e.value}°"
                    self.position_display.update()   
                    send_servo_command(position=e.value)
                    print(f"Slider changed to: {e.value}")  
                    
                
                with ui.element('div').classes('parrot-container w-full flex justify-center items-center relative'):
                    with ui.card().classes('w-full h-full relative'):
                        # Robot parrot interactive control
                        ui.label("Control the Parrot Robot").classes("text-lg font-bold")
                        with ui.element('div').classes('relative mx-auto').style('width: 500px; height: 500px;'):
                            ui.image('static/images/parrot_robot.png').classes('absolute w-full h-full object-contain')

                            # === HEAD ===
                            ui.element('div').classes('hotspot').style('top: 90px; left: 250px;').props('id=head_circle')
                            with ui.element('div').classes('popup').style('top: -20px; left: 280px;').props('id=head_popup'):
                                ui.label('Head Rotate')
                                head_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change).props('label-always')
                                # ui.button('Send', on_click=lambda: self.send_servo('head_rotate', head_slider.value))

                            ui.element('div').classes('hotspot').style('top: 230px; left: 200px;').props('id=tilt_circle')
                            with ui.element('div').classes('popup').style('top: 280px; left: 150px;').props('id=tilt_popup'):
                                ui.label('Head Tilt')
                                head_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change).props('label-always')

                            # === LEFT WING ===
                            ui.element('div').classes('hotspot').style('top: 220px; left: 90px;').props('id=left_circle')
                            with ui.element('div').classes('popup').style('top: 260px; left: -150px;').props('id=left_popup'):
                                ui.label('Left Wing')
                                left_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change).props('label-always')
                                # ui.button('Send', on_click=lambda: self.send_servo('left_wing', left_slider.value))

                            # === RIGHT WING ===
                            ui.element('div').classes('hotspot').style('top: 220px; left: 380px;').props('id=right_circle')
                            with ui.element('div').classes('popup').style('top: 260px; left: 400px;').props('id=right_popup'):
                                ui.label('Right Wing')
                                right_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change).props('label-always')
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
                                servo_status_circle("servo_head_tilt", 90)
                                servo_status_circle(None, "", insert=False)

                                # Row 2: [left wing, empty, right wing]
                                servo_status_circle("servo_left_wing", 170)
                                servo_status_circle(None, "", insert=False)
                                servo_status_circle("servo_right_wing", 5)

                                # Row 3: [empty, head rotate, empty]
                                servo_status_circle(None, "", insert=False)
                                servo_status_circle("servo_head_rotate", 90)
                                servo_status_circle(None, "", insert=False)

                            ui.run_javascript("updateServoStatus('servo_head_tilt', 45, 'moving')")
                            ui.run_javascript("updateServoStatus('servo_right_wing', 0, 'ok')")
                      
                    
                    # position_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=on_slider_change)


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

        # --- Servo command function ---
    def send_servo(self, target, position):
        try:
            url = f"http://192.168.1.42:5000/servo/{target}/position/{position}/method/instant/speed/1.0"
            r = requests.post(url)
            ui.notify(f"{target} moved to {position}°")
        except Exception as e:
            ui.notify(f"Error: {e}")


@ui.page("/")
def main(client: Client):
    MainPage().create_ui()


ui.run(title="Papegaai", favicon='🦜', port=8080, reload=True)