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

    # Always use the main ParrotRobot sounds directory
    # FIXME: Modify accordingly with your path if needed
    sounds_folder = os.path.expanduser("~/ParrotRobot/src/sounds")

    if not os.path.exists(sounds_folder):
        return jsonify({
            "error": f"Sounds folder not found at {sounds_folder}"
        }), 404

    print(f"[SOUND] Listing files in: {sounds_folder}")

    # Collect only valid sound files
    sounds = [
        os.path.splitext(f)[0]
        for f in os.listdir(sounds_folder)
        if os.path.isfile(os.path.join(sounds_folder, f))
        and f.lower().endswith((".mp3", ".wav", ".ogg"))
    ]

    return jsonify({
        "sounds": sounds,
        "count": len(sounds),
        "path": sounds_folder
    })
