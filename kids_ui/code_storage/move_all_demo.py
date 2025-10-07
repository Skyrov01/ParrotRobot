#Start code here
import time
import requests
BASE_URL = "http://192.168.8.121:5000"
HUMAN_DETECTED = True
end_angle = 45.0
start_angle = 120.0
motors = ["left_wing", "right_wing", "head_rotate", "head_tilt"]

while True:
  for i in motors:
    if i == "head_tilt":
      start_angle = 50.0
      end_angle = 170.0
    else:
      start_angle = 45.0
      end_angle = 120.0
    url = f"{BASE_URL}/servo/{i}/position/{start_angle}/method/instant/speed/0.5"
    r = requests.post(url)
    print("Servo:", r)
  time.sleep(1)
  for i in motors:
    url = f"{BASE_URL}/servo/{i}/position/{end_angle}/method/instant/speed/0.5"
    r = requests.post(url)
    print("Servo:", r)
  time.sleep(1)
