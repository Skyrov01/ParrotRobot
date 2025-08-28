# routes/servo_routes.py
from flask import Blueprint, request, jsonify, Response, stream_with_context
from . import context 

from queue import Empty

import json                                          
import time  

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


@servo_bp.route('/servo/cleanup', methods=['POST'])
def servo_cleanup():
    import parrot_robot.servo_driver as servo
    try:
        servo.cleanup()
        return jsonify({"status": "cleanup complete"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

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


@servo_bp.route("/servo/stream", methods=["GET"])
def servo_stream():

    ros_node = context.get_ros_node()

    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    @stream_with_context
    def gen():
        # initial snapshot
        yield f"data: {json.dumps({'snapshot': ros_node.state})}\n\n"
        # incremental updates
        while True:
            try:
                data = ros_node._q.get(timeout=5.0)
                yield f"data: {data}\n\n"
            except Empty:
                yield f": keep-alive {int(time.time())}\n\n"
    headers = {
    "Cache-Control": "no-cache",
    "Connection": "keep-alive",
    "Access-Control-Allow-Origin": "*",  # critical for cross-host EventSource
    }
    return Response(gen(), headers=headers, mimetype="text/event-stream")


# --------------------------------------------------------------------- # 
#                          HIGH LEVEL ROUTES
# --------------------------------------------------------------------- # 

@servo_bp.route('/agree/amplitude/<float:amplitude>/speed/<float:speed>', methods=['POST'])
def agree(amplitude, speed):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503
    msg = BehaviourCommand()
    msg.behaviour_type = "agree"
    msg.amplitude = amplitude
    msg.speed = speed
    ros_node.behaviour_pub.publish(msg)
    return jsonify({"status": "agree command sent"})