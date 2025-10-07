import requests

# 👇 Add the IPs of your parrots here
PARROTS = [
    "http://192.168.8.121:5000",
    "http://192.168.8.125:5000",
    "http://192.168.8.130:5000",
]

# All servo targets
SERVOS = ["head_rotate", "head_tilt", "left_wing", "right_wing"]

for base_url in PARROTS:
    print(f"--- Resetting {base_url} ---")
    for servo in SERVOS:
        url = f"{base_url}/servo/{servo}/position/90.0/method/instant/speed/1.0"
        try:
            r = requests.post(url)
            print(f"{servo} → 90°:", r.json())
        except Exception as e:
            print(f"Error resetting {servo} on {base_url}: {e}")
