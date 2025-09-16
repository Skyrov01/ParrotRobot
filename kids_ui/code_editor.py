from nicegui import ui

from http_client import send_behavior

from http_client import send_servo_command, cleanup
from theme import MAIN_COLOR, ACCENT_COLOR_1, ACCENT_COLOR_2, TEXT_COLOR, WHITE, BLACK, FLAG_IMAGE

import multiprocessing
import os



import time
import sys
import signal
import requests
import threading

BASE_URL = "http://192.168.8.190:5000"


running_process = None
log_queue = multiprocessing.Queue()

CODE_STORAGE_DIR = "code_storage"
os.makedirs(CODE_STORAGE_DIR, exist_ok=True)  # make sure folder existsimport signal
    
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



def worker(code_str, queue):
    """Runs in child process"""
    class QueueWriter:
        def write(self, text):
            if text.strip():
                queue.put(text)
        def flush(self): 
            pass

    sys.stdout = QueueWriter()
    sys.stderr = QueueWriter()

    try:
        exec(code_str, {}, {})
    except Exception as e:
        print(f"Error: {e}")

def code_editor_card():
    with ui.card().classes("w-full"):
        with ui.element().classes("w-full").style("height: 35vh; width: 100%;"):
            editor = ui.codemirror(default_code, language="python", theme="okaidia") \
                    .style("height: 100%; width: 100%;")

        # 20% Output
        with ui.element().classes("w-full h-1/5").style("height: 30%; width: 100%; "):
            output_box = ui.textarea(label="Real-Time Output") \
                        .style("height: 100%; width: 100%; resize: none;")
            
         # --- Dropdown for load ---
        def get_saved_files():
            return [f for f in os.listdir(CODE_STORAGE_DIR) if f.endswith(".py")]

        
        

        # 10% Buttons
        with ui.row().style("height: 10%; width: 100%; align-items: center; justify-content: left;"):
           
            def run():
                global running_process, log_queue
                if running_process and running_process.is_alive():
                    ui.notify("Code already running")
                    return

                output_box.value = ""
                log_queue = multiprocessing.Queue()
                running_process = multiprocessing.Process(
                    target=worker, args=(editor.value, log_queue)
                )
                running_process.start()
                ui.notify(f"Started process with PID {running_process.pid}")
                 # start a log updater thread
                def log_updater():
                    while running_process and running_process.is_alive():
                        try:
                            line = log_queue.get(timeout=0.2)
                            output_box.value += line + "\n"
                            output_box.update()
                        except Exception:
                            pass
                        time.sleep(0.1)

                threading.Thread(target=log_updater, daemon=True).start()


            def kill():
                global running_process
                if running_process and running_process.is_alive():
                    os.kill(running_process.pid, signal.SIGTERM)
                    running_process.join(timeout=1)
                    ui.notify("Process killed")
                else:
                    ui.notify("No process running")


            def cleanup():
                global running_process
                # 1. Kill running process if alive
                if running_process and running_process.is_alive():
                    os.kill(running_process.pid, signal.SIGTERM)
                    running_process.join(timeout=1)

                # 2. Reset editor and output
                editor.value = default_code
                output_box.value = ""
                editor.update()
                output_box.update()

                # 3. Call servo cleanup endpoint
                try:
                    url = f"{BASE_URL}/servo/cleanup"
                    r = requests.post(url)
                    result = r.json()
                    ui.notify(result.get("status", "No response"))
                except Exception as e:
                    ui.notify(f"Servo cleanup failed: {e}")

            def save_code():
                name = code_name_input.value.strip()
                if not name:
                    ui.notify("Please enter a name for your code")
                    return

                # Ensure filename safe
                safe_name = "".join(c for c in name if c.isalnum() or c in ("_", "-"))
                filepath = os.path.join(CODE_STORAGE_DIR, f"{safe_name}.py")

                try:
                    with open(filepath, "w") as f:
                        f.write(editor.value)
                    ui.notify(f"Code saved as {filepath}")
                except Exception as e:
                    ui.notify(f"Save failed: {e}")


            def load_code():
                filename = load_dropdown.value
                if not filename:
                    ui.notify("Please select a file to load")
                    return

                filepath = os.path.join(CODE_STORAGE_DIR, filename)
                try:
                    with open(filepath, "r") as f:
                        editor.value = f.read()
                    editor.update()
                    ui.notify(f"Loaded {filename}")
                except Exception as e:
                    ui.notify(f"Load failed: {e}")

            with ui.row().classes("gap-2 mt-2 w-full justify-between items-center"):
                # Left group
                with ui.row().classes("gap-2"):
                    ui.button("Run Code", on_click=run, color="jungle")
                    ui.button("Kill Code", on_click=kill, color="red")
                    ui.button("Cleanup", on_click=cleanup, color="sunrise")

                # Right group (name + save)
                with ui.row().classes("gap-2 items-center"):
                    code_name_input = ui.input("Save your file ...").classes("w-60 mb-4 mr-2")
                    ui.button("Save Code", on_click=save_code, color="jungle")
                    load_dropdown = ui.select(get_saved_files(), label="Load file").classes("w-40 mb-4 mr-2")
                    ui.button("Load Code", on_click=load_code, color="jungle")

           
        