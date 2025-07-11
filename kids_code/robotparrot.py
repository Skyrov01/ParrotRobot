#!/usr/bin/env python3

#Start code here
import requests
r = requests.post("http://192.168.1.126:5000/nod/fast/3")
print(r)

import requests

r = requests.post("http://192.168.1.126:5000/servo/head/90.0/3.0")

print("Status Code:", r.status_code)
print("Response JSON:", r.json())