

from nicegui import Client, app, ui

import threading
import sys
import time

import requests
from threading import Lock, Thread

# These two are files where html, css and javascript are defined
from style import *
from script import *

# Custom imports
from ui_components import add_navigation
from ui_components import behaviour_card
from ui_components import robot_setup_card, advanced_control_card, servo_status_card, description_card

from code_editor import code_editor_card

from http_client import send_servo_command, send_behavior, cleanup
from config import *

from theme import *




BASE_URL = "http://192.168.8.190:5000"

def generate_route_code(path_template: str, variables: dict):
    
    route = path_template
    for key, value in variables.items():
        route = route.replace(f"<{key}>", str(value))
    full_url = f"{BASE_URL}/{route}"
    code = f'requests.post("{full_url}")'
    return route, code

ui.add_css(style_parrot_control)
ui.add_css(style_navbar)


class MainPage:
    def __init__(self):
        pass
        self.dark_mode = False
        self.text_color = "text-white" if self.dark_mode else "text-gray-800"
        self.subtext_color = "text-gray-400" if self.dark_mode else "text-gray-600"
        self.accent_color = "text-purple-500"
        self.position_display = None

    def howto_section(self, title, command, description, topics):
        with ui.expansion(title, icon="rocket", value=False).classes("w-full"):
            ui.code(command)
            ui.label(description).classes(f"mb-2 {self.subtext_color}")
            ui.label(f"Topics: {topics}").classes(f"mb-2 {self.subtext_color}")

    def create_ui(self):
        ui.colors(
            jungle=MAIN_COLOR,          # parrot green, main brand color
            sunset=ACCENT_COLOR_1,      # bright red/orange, action/alert
            sunrise=ACCENT_COLOR_2,     # warm yellow, highlight
            feather=TEXT_COLOR,         # dark gray text
            sky=WHITE,                  # off-white background
            midnight=BLACK,             # black background
            success="#00bf63",        # success green
            danger="#e85642",         # error red
            ocean="#3da9fc",          # info blue
            sand="#fbce37",           # warning yellow
        )

    
        with ui.column().style("padding: 0; gap: 0;").classes("w-full h-full"):
            code_editor_card()

            with ui.card().classes("w-full mt-6"):
                
                


                with ui.row().classes("w-full gap-1 no-wrap"):
                    robot_setup_card(lambda new=None: BASE_URL if new is None else globals().update(BASE_URL=new))
            
               # Container for the parrot image and controls
                ui.add_head_html(style_robot_controls_tailwind)
                ui.add_head_html(script_status_servo)
                # --- JavaScript for toggling popups on click ---
                ui.add_body_html(script_robot_controls_tailwind)

                def on_slider_change(target, e):
                    ui.notify("Slider changed", color="sunset")
                    send_servo_command(target=target, position=float(e.value))
                    
                
                with ui.element("div").classes("parrot-container w-full flex justify-center items-center relative"):
                    with ui.card().classes("w-full h-full relative"):

                        def servo_status_circle(id, angle, top, left, insert=True):
                            circle = ui.element("div").classes("hotspot").style(f"top: {top}px; left: {left}px;") \
                                .classes(
                                    "rounded-full w-10 h-10 flex items-center justify-center text-white "
                                    "font-bold text-sm border-4 border-white shadow-md transition-all"
                                ).style("background-color: gray;").props(f"id={id}")
                            if not insert:
                                circle.classes("invisible hotspot")
                            return circle
                        
                        # Robot parrot interactive control
                        ui.label("Manual Control").classes("text-lg font-bold")
                        with ui.element("div").classes("relative mx-auto").style("width: 500px; height: 500px;"):
                            ui.image("static/images/parrot_robot.png").classes("absolute w-full h-full object-contain")

                            # === HEAD ===
                            servo_status_circle("servo_head_rotate", 90, 90, 220)
                            # ui.element("div").classes("hotspot").style("top: 90px; left: 250px;").props("id=head_circle")
                            with ui.element("div").classes("popup").style("top: -20px; left: 280px;").props("id=head_popup"):
                                ui.label("Head Rotate")
                                head_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("head_rotate", e)).props("label-always")
                                # ui.button("Send", on_click=lambda: self.send_servo("head_rotate", head_slider.value))

                            servo_status_circle("servo_head_tilt", 90, 230, 220)
                            with ui.element("div").classes("popup").style("top: 280px; left: 150px;").props("id=tilt_popup"):
                                ui.label("Head Tilt")
                                head_slider = ui.slider(min=10, max=170, value=90, step=1,on_change=lambda e: on_slider_change("head_tilt", e)).props("label-always")

                            # === LEFT WING ===
                            servo_status_circle("servo_left_wing", 90, 220, 70)
                            with ui.element("div").classes("popup").style("top: 260px; left: -150px;").props("id=left_popup"):
                                ui.label("Left Wing")
                                left_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("left_wing", e)).props("label-always")
                                # ui.button("Send", on_click=lambda: self.send_servo("left_wing", left_slider.value))

                            # === RIGHT WING ===
                            servo_status_circle("servo_right_wing", 90, 220, 370)
                            with ui.element("div").classes("popup").style("top: 260px; left: 400px;").props("id=right_popup"):
                                ui.label("Right Wing")
                                right_slider = ui.slider(min=10, max=170, value=90, step=1, on_change=lambda e: on_slider_change("right_wing", e)).props("label-always")
                                # ui.button("Send", on_click=lambda: self.send_servo("right_wing", right_slider.value))

                            
                

                with ui.card().classes("w-full mt-6"):

                        
                    with ui.row().classes("w-full gap-1 no-wrap"):
                        output_area = advanced_control_card()
                        

                # -------------------------- Predefined Motions -----------------------------

                # Agree Card
                with ui.row().classes("w-full gap-1 no-wrap"):
                    # Agree
                    behaviour_card(
                        "Agree",
                        lambda amp, speed, reps: f"/agree/amplitude/{amp}/speed/{speed}/repetitions/{reps}",
                        defaults={"amp": "medium", "speed": 1.0, "reps": 1},
                    )

                    # Disagree
                    behaviour_card(
                        "Disagree",
                        lambda amp, speed, reps: f"/disagree/amplitude/{amp}/speed/{speed}/repetitions/{reps}",
                        defaults={"amp": "medium", "speed": 1.0, "reps": 1},
                    )

                with ui.row().classes("w-full gap-1 no-wrap"):
                    # Maybe (Indifference)
                    behaviour_card(
                        "Maybe",
                        lambda amp, speed, reps: f"/maybe/amplitude/{amp}/speed/{speed}/repetitions/{reps}",
                        defaults={"amp": "medium", "speed": 1.0, "reps": 1},
                    )

                    # Wave
                    behaviour_card(
                        "Wave",
                        lambda wing, amp, speed, reps: f"/wave/{wing}/amplitude/{amp}/speed/{speed}/repetitions/{reps}",
                        defaults={"wing": "left_wing", "amp": "medium", "speed": 1.0, "reps": 3},
                        include_wing=True,
                    )


                def update_servo_ui(id, angle, status):
                    color = {
                        "ok": "green",
                        "moving": "orange",
                        "error": "red"
                    }.get(status, "gray")
                    ui.run_javascript(f"""
                        const el = document.getElementById("{id}");
                        if (el) {{
                            el.innerText = "{angle}";
                            el.style.backgroundColor = "{color}";
                        }}
                    """)


@ui.page("/")
def main(client: Client):
    add_navigation(current="/") 
    MainPage().create_ui()

@ui.page("/editor")
def editor_page(client: Client):
    add_navigation(current="/editor")
    ui.label("Python Code Editor Here")

@ui.page("/control")
def control_page(client: Client):
    add_navigation(current="/control")
    ui.label("Parrot Robot Control Interface")

@ui.page("/docs")
def documentation_page(client: Client):
    add_navigation(current="/docs")
    ui.label("Robot Documentation Page")

@ui.page("/settings")
def settings_page(client: Client):
    add_navigation(current="/settings")
    ui.label("Robot Settings Page")

ui.run(title="Papegaai", favicon="🦜", port=8080, reload=True)