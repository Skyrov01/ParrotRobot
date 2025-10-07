# Start code here
import time
import requests

PARROTS = [
    "http://192.168.8.121:5000",  # Parrot 1
    "http://192.168.8.125:5000",  # Parrot 2
    "http://192.168.8.130:5000",  # Parrot 3
]

HUMAN_DETECTED = True

def send_all(path):
    """Helper: send the same request to all parrots"""
    for base_url in PARROTS:
        url = f"{base_url}{path}"
        try:
            r = requests.post(url)
            print(f"[{base_url}] -> {path}: {r.status_code}")
        except Exception as e:
            print(f"[{base_url}] ERROR: {e}")

while not HUMAN_DETECTED:
    speed = 0.5
    selected_sound = "whistle"
    send_all(f"/lookaround/speed/{speed}")
    send_all(f"/sound/play/{selected_sound}")
    time.sleep(1)

how_many_times = 5
selected_sound = "greetings"
send_all(f"/sound/play/{selected_sound}")

end_angle = 45.0
start_angle = 120.0

print("Humaaaaaan!")

for i in range(25):
    print("Nodding")
    send_all(f"/servo/head_rotate/position/{start_angle}/method/instant/speed/0.5")
    send_all(f"/servo/left_wing/position/{start_angle}/method/instant/speed/0.5")
    time.sleep(1)

    send_all(f"/servo/head_rotate/position/{end_angle}/method/instant/speed/0.5")
    send_all(f"/servo/left_wing/position/{end_angle}/method/instant/speed/0.5")
    time.sleep(1)

    if i == round(how_many_times / 2):
        send_all(f"/sound/play/{selected_sound}")
