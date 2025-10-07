# Test
import time
import requests

BASE_URL = "http://192.168.8.121:5000"

def move_head(position, speed=1.0):
    global BASE_URL
    url = f"{BASE_URL}/servo/head_rotate/position/{position}/method/instant/speed/{speed}"
    r = requests.post(url)
    print(f"Head to {position}°:", r.json())

def move_wings(position, speed=1.0):
    global BASE_URL
    # Left wing
    url_left = f"{BASE_URL}/servo/left_wing/position/{position}/method/instant/speed/{speed}"
    r1 = requests.post(url_left)
    print("Left wing:", r1.json())

    # Right wing
    url_right = f"{BASE_URL}/servo/right_wing/position/{position}/method/instant/speed/{speed}"
    r2 = requests.post(url_right)
    print("Right wing:", r2.json())

def flap_wings(up=170.0, down=10.0, reps=3, speed=1.0):
    for i in range(reps):
        move_wings(up, speed)
        time.sleep(0.5)
        move_wings(down, speed)
        time.sleep(0.5)

while True:
    move_head(45.0, speed=0.8)   # left
    time.sleep(1)

    move_head(90.0, speed=0.8)   # middle
    flap_wings(reps=3, speed=1.0)
    time.sleep(1)

    move_head(135.0, speed=0.8)  # right
    time.sleep(1)

    move_head(90.0, speed=0.8)   # back to middle
    time.sleep(1)
