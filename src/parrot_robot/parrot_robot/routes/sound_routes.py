# routes/sound_routes.py
from flask import Blueprint, jsonify
from . import context
from parrot_msgs.msg import SoundCommand

import os

sound_bp = Blueprint("sound", __name__)

@sound_bp.route("/sound/play/<sound_name>", methods=["POST"])
def play_sound(sound_name):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    msg = SoundCommand()
    msg.sound_name = sound_name
    ros_node.sound_pub.publish(msg)

    return jsonify({"status": "playing", "sound": sound_name})



# --- Stop current sound ---
@sound_bp.route("/sound/stop", methods=["POST"])
def stop_sound():
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    msg = SoundCommand()
    msg.action = "stop"
    ros_node.sound_pub.publish(msg)

    return jsonify({"status": "stopped"})


# --- List available sounds ---
@sound_bp.route("/sound/list", methods=["GET"])
def list_sounds():
    # Path to the folder containing the sound files
    # Try local project structure first
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    candidate_paths = [
        os.path.join(project_root, "sounds"),                         # e.g. /home/parrot/work/parrot/ParrotRobot/sounds
        os.path.join(os.path.dirname(__file__), "..", "sounds"),       # fallback for relative import
        "/home/parrot/work/parrot/ParrotRobot/src/sounds",             # explicit known path if using ROS2 install
    ]

    sounds_folder = next((p for p in candidate_paths if os.path.exists(p)), None)

    if not sounds_folder:
        return jsonify({"error": "Sounds folder not found in known locations", "checked": candidate_paths}), 404

    print(f"Listing sounds in folder: {sounds_folder}")
    # Collect sound file names without extensions
    sounds = [
        os.path.splitext(f)[0]
        for f in os.listdir(sounds_folder)
        if os.path.isfile(os.path.join(sounds_folder, f))
           and f.lower().endswith((".mp3", ".wav", ".ogg"))
    ]

    return jsonify({"sounds": sounds, "count": len(sounds)})