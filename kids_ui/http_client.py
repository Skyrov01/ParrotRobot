import requests

BASE_URL = "http://192.168.8.190:5000"

def send_servo_command(target, position, method="instant", speed=1.0):
    url = f"{BASE_URL}/servo/{target}/position/{position}/method/{method}/speed/{speed}"
    return _post(url)

def send_behavior(route):
    url = f"{BASE_URL}{route}"
    return _post(url)

def cleanup():
    url = f"{BASE_URL}/servo/cleanup"
    return _post(url)

def _post(url):
    try:
        r = requests.post(url)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        return {"error": str(e)}