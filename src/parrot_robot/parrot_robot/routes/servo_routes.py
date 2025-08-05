# routes/servo_routes.py
from flask import Blueprint, request, jsonify
from . import context 

servo_bp = Blueprint("servo", __name__)

@servo_bp.route('/servo/<target>/position/<float:position>/method/<method>/speed/<float:speed>', methods=['POST'])
def servo_move(target, position, method, speed):
    ros_node = context.get_ros_node()

    position = float(position)
    speed = float(speed)

    if ros_node:
        ros_node.publish_servo(target, position, speed, method)
        return jsonify({
            "status": "servo command sent",
            "target": target,
            "position": position,
            "method": method,
            "speed": speed
        })
    else:
        return jsonify({"status": "ROS not ready"}), 503

@servo_bp.route('/servo/wings/flap/left/<float:left>/right/<float:right>/method/<method>/speed/<float:speed>/reps/<int:reps>', methods=['POST'])
def flap_wings(left, right, method, speed, reps):
    ros_node = context.get_ros_node()
    if ros_node:
        ros_node.publish_wings(left, right, speed, method, reps)
        return jsonify({
            "status": "flap wings command sent",
            "left": left,
            "right": right,
            "speed": speed,
            "method": method,
            "repetitions": reps
        })
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


@servo_bp.route('/servo/<target>/status', methods=['GET'])
def servo_status(target):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    status = ros_node.get_servo_status(target)
    if status is None:
        return jsonify({"status": "unknown target"}), 404

    return jsonify({
        "target": target,
        "busy": status
    })