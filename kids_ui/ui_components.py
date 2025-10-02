# ui_components.py
from nicegui import ui, app

from http_client import send_behavior

from http_client import send_servo_command, cleanup
from theme import MAIN_COLOR, ACCENT_COLOR_1, ACCENT_COLOR_2, TEXT_COLOR, WHITE, BLACK, FLAG_IMAGE

BASE_URL = "http://192.168.8.120:5000"
# Navigation bar 

def add_navigation(dark_mode: bool = False, current: str = ""):
    is_dark = dark_mode

    

    def nav_link(label: str, path: str, special=False):
        is_active = current == path
        base = "px-4 py-2 rounded-md font-bold no-underline transition-colors"

        

        # Normal items
        if is_dark:
            active_classes = f"bg-[{MAIN_COLOR}] text-[{WHITE}]"
            inactive_classes = f"bg-[{BLACK}] hover:bg-[{ACCENT_COLOR_1}] text-[{ACCENT_COLOR_2}]"
        else:
            active_classes = f"bg-[{MAIN_COLOR}] text-[{WHITE}]"
            inactive_classes = f"bg-[{WHITE}] hover:bg-[{ACCENT_COLOR_2}] text-[{TEXT_COLOR}]"


        return ui.link(label, path).classes(
            f"{base} {active_classes if is_active else inactive_classes}"
        )

    with ui.header().classes(
        "p-4 sticky top-0 z-50 shadow-md flex-shrink-0 " +
        (f"bg-[{BLACK}] text-[{WHITE}]" if is_dark else f"bg-[{WHITE}] text-[{TEXT_COLOR}]")
    ):
        with ui.row().classes("items-center justify-between w-full"):
            # Project Title
            ui.label("Papegaai Robot").classes(
                "text-2xl font-bold " + (f"text-[{ACCENT_COLOR_2}]" if is_dark else f"text-[{MAIN_COLOR}]"))


            # Navigation links
            with ui.row().classes("gap-4 items-center"):
                nav_link("Home", "/", special=True)
                nav_link("Code Editor", "/editor")
                nav_link("Robot Control", "/control")
                nav_link("Documentation", "/docs")
                nav_link("Settings", "/settings")

def description_card():
    with ui.card().classes("w-full"):
        ui.label("Robot Description").classes("text-lg font-bold")
        with ui.column().classes("gap-2"):
            ui.label("Papegaai is a small social robot in the form of a parrot.").classes("text-base")
            ui.label("It was designed together with 12-year-old children during workshops in Namibia.").classes("text-base")
            ui.label("The robot has 4 main servo motors:").classes("font-semibold mt-2")
            ui.label("• Head Rotate – turns the head left and right")
            ui.label("• Head Tilt – moves the head up and down")
            ui.label("• Left Wing – flaps the left wing")
            ui.label("• Right Wing – flaps the right wing")

            ui.label("The robot is controlled via ROS2 and an HTTP server bridge.").classes("mt-2")
            ui.label("It can perform behaviours such as Agree, Disagree, Wave, and Look Around.")



def behaviour_card(name: str, route_builder, defaults: dict = None, include_wing=False):
    """
    A themed reusable card for predefined behaviours (Agree, Disagree, Maybe, Wave).
    """
    defaults = defaults or {}

    with ui.card().classes("w-full md:w-1/2 shadow-lg").props("color=sky text-feather"):
        ui.label(f"{name} Motion").classes("text-lg font-bold").props("text-jungle")

        # --- Wing selector (for Wave only) ---
        wing = None
        if include_wing:
            with ui.row().classes("items-center gap-4 w-full"):
                ui.label("Wing").classes("w-24").props("text-feather")
                wing = ui.select(
                    ["left_wing", "right_wing"], 
                    value=defaults.get("wing", "left_wing")
                ).classes("w-1/2 flex-grow").props("color=jungle")

        # --- Amplitude ---
        with ui.row().classes("items-center gap-4 w-full"):
            ui.label("Amplitude").classes("w-24").props("text-feather")
            amp = ui.toggle(["low", "medium", "high"], value=defaults.get("amp", "medium")) \
                .props("color=feather spread toggle-color=jungle") \
                .classes("w-1/2 flex-grow")

        # --- Speed ---
        with ui.row().classes("items-center gap-4 w-full"):
            ui.label("Speed").classes("w-24").props("text-feather")
            speed = ui.slider(min=0.1, max=5.0, value=defaults.get("speed", 1.0), step=0.1) \
                .props("label-always color=jungle") \
                .classes("w-1/2 flex-grow")

        # --- Repetitions ---
        with ui.row().classes("items-center gap-4 w-full"):
            ui.label("Repetitions").classes("w-24").props("text-feather")
            reps = ui.slider(min=1, max=10, value=defaults.get("reps", 1), step=1) \
                .props("label-always color=jungle") \
                .classes("w-1/2 flex-grow")

        # --- Preview fields ---
        route_label = ui.label().classes("mt-2 text-sm font-mono").props("text-sunset")
        code_box = ui.input(label="Python Code").classes("w-full h-8 font-mono").props("color=sky text-feather")

        # --- Internal updater ---
        def update_preview():
            if include_wing:
                route = route_builder(wing.value, amp.value, speed.value, int(reps.value))
            else:
                route = route_builder(amp.value, speed.value, int(reps.value))

            full_code = f'requests.post(f"{{{app.storage.user["robot_ip"]}}}{route}")'

            route_label.text = f"Route: {route}"
            code_box.value = full_code
            return route

        # --- Execute button ---
        def execute():
            route = update_preview()
            data = send_behavior(route)
            if "error" in data:
                ui.notify(f"❌ {name} failed: {data['error']}", color="danger")
            else:
                ui.notify(f"✅ {name} executed: {data}", color="jungle")

        # Bind updates
        if include_wing:
            wing.on('update:model-value', lambda e: update_preview())
        amp.on('update:model-value', lambda e: update_preview())
        speed.on('update:model-value', lambda e: update_preview())
        reps.on('update:model-value', lambda e: update_preview())

        ui.button("Send Command", on_click=execute).classes("mt-4 font-bold").props("color=jungle")

        # Initialize preview
        update_preview()

# -------------------- Robot Setup Card --------------------
def robot_setup_card(base_url_ref):
    with ui.card().classes("w-1/2"):
        ui.label("Robot Setup Parameters").classes("text-lg font-bold")

        ui.label("Base URL").classes("mt-2")
        base_url_input = ui.input(value=base_url_ref()).classes("w-full").props('outlined')

        def update_base_url():
            new_url = base_url_input.value
            base_url_ref(new_url)

            # Save the IP to persistent storage
            app.storage.user["robot_ip"] = new_url

            ui.notify(f"✅ BASE_URL updated to {new_url}")

        base_url_input.on('blur', lambda e: update_base_url())
        base_url_input.on('keydown.enter', lambda e: update_base_url())
        ui.button("Save IP", on_click=update_base_url, color="jungle").classes("mt-2")


# -------------------- Advanced Control Card --------------------



def advanced_control_card():
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

        output_area = ui.textarea(label="Response").classes("w-full h-32")
        output_area.value = "Output will be displayed here"

        def send():
            data = send_servo_command(
                base_url=app.storage.user.get["robot_ip"], 
                target=target_input.value,
                position=position_input.value,
                method=method_input.value,
                speed=speed_input.value,
            )
            if "error" in data:
                output_area.value = f"❌ Error:\n{data['error']}"
                ui.notify("Failed to send command")
            else:
                output_area.value = f"✅ Success:\n{data}"
                ui.notify("Command sent!")

        ui.button("Send Command", on_click=send, color="sunrise").classes("mt-2")

    return output_area


# -------------------- Servo Status Card --------------------
def servo_status_card():
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


