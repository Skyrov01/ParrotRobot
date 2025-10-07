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
    

def play_sound(base_url, sound_name):
    """
    Play a specific sound on the robot.
    Example: play_sound("http://192.168.8.121:5000", "whistle")
    """
    url = f"{base_url}/sound/play/{sound_name}"
    return _post(url)


def stop_sound(base_url):
    """
    Stop any currently playing sound.
    Example: stop_sound("http://192.168.8.121:5000")
    """
    url = f"{base_url}/sound/stop"
    return _post(url)


def list_sounds(base_url):
    """
    Get a list of available sounds from the robot.
    Example: list_sounds("http://192.168.8.121:5000")
    """
    url = f"{base_url}/sound/list"
    try:
        r = requests.get(url)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        return {"error": str(e)}