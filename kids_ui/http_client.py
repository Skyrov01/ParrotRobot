import requests


def send_servo_command(target, base_url, position, method="instant", speed=1.0):
    url = f"{base_url}/servo/{target}/position/{position}/method/{method}/speed/{speed}"
    return _post(url)

def send_behavior(base_url, route):
    url = f"{base_url}{route}"
    return _post(url)

def cleanup(base_url):
    url = f"{base_url}/servo/cleanup"
    return _post(url)

def _post(url):
    try:
        r = requests.post(url)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        return {"error": str(e)}