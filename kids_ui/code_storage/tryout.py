import time
import requests

BASE_URL = "http://192.168.8.190:5000"
HUMAN_DETECTED = False

def lookaround(speed):
    global BASE_URL
    url = f"{BASE_URL}/lookaround/speed{speed}/repetitions/1"
    r = requests.post(url)
    print("Lookaround:", r)
    time.sleep(1)
    
while not HUMAN_DETECTED:
    lookaround(0.5)
