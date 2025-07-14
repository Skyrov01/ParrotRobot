# routes/servo_routes.py
from flask import Blueprint, request, jsonify
from . import context 

servo_bp = Blueprint("servo", __name__)

@servo_bp.route('/servo/<target>/position/<float:position>/speed/<float:speed>', methods=['POST'])
def servo_move(target, position, speed):
  
    ros_node = context.get_ros_node() 
    # Ensure position and speed are floats
    position = float(position)
    speed = float(speed)

    if ros_node:
        ros_node.publish_servo(target, position, speed)
        return jsonify({"status": "servo command sent", "target": target, "position": position, "speed": speed})
    else:
        return jsonify({"status": "ROS not ready"}), 503

@servo_bp.route('/servo/<target>/reset', methods=['POST'])
def servo_reset(target):
    ros_node = context.get_ros_node() 

    if ros_node:
        ros_node.publish_servo(target, 90.0, 1.0)  # Default reset
        return jsonify({"status": "servo reset", "target": target})
    else:
        return jsonify({"status": "ROS not ready"}), 503



@servo_bp.route('/servo/debug', methods=['GET'])
def debug_ros_node():
    ros_node = context.get_ros_node() 

    return jsonify({"ros_node": str(ros_node), "type": str(type(ros_node))})