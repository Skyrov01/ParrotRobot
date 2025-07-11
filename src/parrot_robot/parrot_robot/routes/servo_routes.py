# routes/servo_routes.py
from flask import Blueprint, request, jsonify
from .context import ros_node  

servo_bp = Blueprint("servo", __name__)


@servo_bp.route('/servo/<target>/<float:position>/<float:speed>', methods=['POST'])
def servo_move(target, position, speed):
    if ros_node:
        ros_node.publish_servo(target, position, speed)
        return jsonify({"status": "servo command sent", "target": target, "position": position, "speed": speed})
    else:
        return jsonify({"status": "ROS not ready"}), 503

@servo_bp.route('/servo/<target>/reset', methods=['POST'])
def servo_reset(target):
    if ros_node:
        ros_node.publish_servo(target, 90.0, 1.0)  # Default reset
        return jsonify({"status": "servo reset", "target": target})
    else:
        return jsonify({"status": "ROS not ready"}), 503