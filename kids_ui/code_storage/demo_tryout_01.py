#Start code here
import time
import requests
BASE_URL = "http://192.168.8.190:5000"
HUMAN_DETECTED = True
while not HUMAN_DETECTED:
  speed = 0.5
  selected_sound = "whistle"
  url = f"{BASE_URL}/lookaround/speed/{speed}"
  r = requests.post(url)
  print("Lookaround:", r)
  url = f"{BASE_URL}/sound/play/{selected_sound}"
  r = requests.post(url)
  print("Sounds", r)
  time.sleep(1)
how_many_times = 5
selected_sound = "greetings"
url = f"{BASE_URL}/sound/play/{selected_sound}"
r = requests.post(url)
print("Sounds:", r)
end_angle = 45.0
start_angle = 120.0
for i in range(10):
  print("Nodding")
  url = f"{BASE_URL}/servo/head_rotate/position/{start_angle}/method/instant/speed/0.5"
  r = requests.post(url)
  print("Servo:", r)
  time.sleep(1)
  url = f"{BASE_URL}/servo/head_rotate/position/{end_angle}/method/instant/speed/0.5"
  r = requests.post(url)
  print("Servo:", r)
  time.sleep(1)
  if i == round(how_many_times / 2):
    url = f"{BASE_URL}/sound/play/{selected_sound}"
    r = requests.post(url)
    print("Sounds:", r)
