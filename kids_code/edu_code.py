#Start code here
import time
import requests
target = "head_rotate"
left_angle = 10.0
right_angle = 170.0
speed = 1.0
host = "http://192.168.1.42:5000"
delay = 0.5
servo_url_left = f"{host}/servo/{target}/position/{left_angle}/speed/{speed}"
print(servo_url_left)
servo_url_right = f"{host}/servo/{target}/position/{right_angle}/speed/{speed}"
print(servo_url_right)
for i in range(4):
  r = requests.post(servo_url_left)
  print(r)
  time.sleep(delay)
  r = requests.post(servo_url_right)
  print(r)
  time.sleep(delay)

