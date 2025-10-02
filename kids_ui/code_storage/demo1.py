import time
import requests

BASE_URL = "http://192.168.8.190:5000"

# Simulated human detection
HUMAN_DETECT = False


def play_sound(sound_name="hello"):
    # assuming you have a /sound/<sound_name> route
    url = f"{BASE_URL}/sound/{sound_name}"
    r = requests.post(url)
    print(f"Play sound '{sound_name}':", r)
    return r


def look_around(speed=1.0):
    url = f"{BASE_URL}/lookaround/speed/{speed}"
    r = requests.post(url)
    print("Lookaround:", r)
    return r


if __name__ == "__main__":
    print("Starting parrot test...")

    # Play a startup sound
    play_sound("hello")

    # Look around until human detected
    while not HUMAN_DETECT:
        print("No human detected → looking around...")
        look_around(speed=1.0)
        time.sleep(2)  # short pause before repeating

    print("Human detected! Stopping lookaround.")
    play_sound("detected")