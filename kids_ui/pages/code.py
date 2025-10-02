import sys
import threading
import requests

from nicegui import Client, app, ui
from config import BASE_URL
from theme import *

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
class CodePage:
    def __init__(self):
        pass

    
        
    def create_ui(self):
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

                ui.button("Run Code", on_click=run, color="jungle").classes()
                ui.button("Cleanup", on_click=cleanup, color="sunrise")