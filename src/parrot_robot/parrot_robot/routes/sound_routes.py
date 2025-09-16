# routes/sound_routes.py
from flask import Blueprint, jsonify
from . import context
from parrot_msgs.msg import SoundCommand

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