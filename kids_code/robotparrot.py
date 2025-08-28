#!/usr/bin/env python3

import requests
import time

target = "head_rotate"
left = 10.0
right = 170.0
speed = 1.0
repeat = 4
delay = 0.5
host = "192.168.1.42"

# Create URLs manually
url_left = f"http://{host}:5000/servo/{target}/position/{left}/speed/{speed}"
url_right = f"http://{host}:5000/servo/{target}/position/{right}/speed/{speed}"

for i in range(repeat):
    print(f"🔄 Cycle {i+1}: rotating head left")
    try:
        response = requests.post(url_left)
        print("→", response.status_code, response.json())
    except requests.exceptions.JSONDecodeError:
        print("→", response.status_code, response.text)
    time.sleep(delay)

    print(f"🔄 Cycle {i+1}: rotating head right")
    try:
        response = requests.post(url_right)
        print("→", response.status_code, response.json())
    except requests.exceptions.JSONDecodeError:
        print("→", response.status_code, response.text)
    time.sleep(delay)

print("✅ Done rotating head 4 times.")

