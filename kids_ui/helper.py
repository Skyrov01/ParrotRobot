import sys, threading

class RealTimeOutput:
    def __init__(self, output_box):
        self.output_box = output_box
    def write(self, text):
        if text.strip():
            self.output_box.value += text + "\n"
            self.output_box.update()
    def flush(self): pass

def run_user_code(code, output_box):
    def worker():
        output_box.value = ""
        output = RealTimeOutput(output_box)
        old_stdout = sys.stdout
        sys.stdout = output
        try:
            exec(code, {}, {})
        except Exception as e:
            print(f"Error: {e}")
        finally:
            sys.stdout = old_stdout
    threading.Thread(target=worker).start()
