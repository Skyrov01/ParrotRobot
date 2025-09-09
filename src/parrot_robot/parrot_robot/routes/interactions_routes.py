from flask import Flask, request, Response, jsonify, Blueprint
from . import context 

interactions_bp = Blueprint("interactions", __name__)


@interactions_bp.route('/nod/<speed>/<int:count>', methods=['POST'])
def nod_route(speed, count):
    
    # Here you could publish a NodCommand
    return jsonify({
        "status": "nod triggered",
        "speed": speed,
        "count": count
    })


# --------------------------------------------------------------------- # 
#                          HIGH LEVEL ROUTES
# --------------------------------------------------------------------- #

@interactions_bp.route('/agree/amplitude/<amplitude>/speed/<float:speed>/repetitions/<int:repetitions>', methods=['POST'])
def agree_route(amplitude, speed, repetitions):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    ros_node.publish_behaviour(
        behaviour_type="agree",
        amplitude=amplitude,   # string: low | medium | high
        speed=speed,
        repetitions=repetitions
    )

    return jsonify({
        "status": "agree command published",
        "amplitude": amplitude,
        "speed": speed,
        "repetitions": repetitions
    })
    
@interactions_bp.route('/disagree/amplitude/<amplitude>/speed/<float:speed>/repetitions/<int:repetitions>', methods=['POST'])
def disagree(amplitude, speed, repetitions):
    """Shake head left–right and lift wings to show disagreement"""
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    # Publish behaviour command
    ros_node.publish_behaviour(
        behaviour_type="disagree",
        amplitude=amplitude,   # "low", "medium", or "high"
        speed=speed,
        repetitions=repetitions
    )

    return jsonify({
        "status": "disagree command published",
        "behaviour": "disagree",
        "amplitude": amplitude,
        "speed": speed,
        "repetitions": repetitions
    })

@interactions_bp.route('/maybe/amplitude/<amplitude>/speed/<float:speed>/repetitions/<int:repetitions>', methods=['POST'])
def maybe(amplitude, speed, repetitions):
    """Shake head left–right and lift wings to show maybe"""
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    # Publish behaviour command
    ros_node.publish_behaviour(
        behaviour_type="maybe",
        amplitude=amplitude,   # "low", "medium", or "high"
        speed=speed,
        repetitions=repetitions
    )

    return jsonify({
        "status": "maybe command published",
        "behaviour": "maybe",
        "amplitude": amplitude,
        "speed": speed,
        "repetitions": repetitions
    })


@interactions_bp.route('/wave/<wing>/amplitude/<amplitude>/speed/<float:speed>/repetitions/<int:repetitions>', methods=['POST'])
def wave(wing, amplitude, speed, repetitions):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    ros_node.publish_behaviour("wave", amplitude=amplitude, speed=speed, repetitions=repetitions, target=wing)
    return jsonify({
        "status": "wave command published", 
        "wing": wing, 
        "repetitions": repetitions, 
        "amplitude": amplitude, 
        "speed": speed
        
        })


@interactions_bp.route('/lookaround/speed/<float:speed>/repetitions/<int:repetitions>', methods=['POST'])
def lookaround(speed, repetitions):
    ros_node = context.get_ros_node()
    if not ros_node:
        return jsonify({"status": "ROS not ready"}), 503

    # Publish behaviour command
    ros_node.publish_behaviour(
        behaviour_type="lookaround",
        speed=speed,
        repetitions=repetitions
    )

    return jsonify({
        "status": "lookaround command published",
        "behaviour": "lookaround",
        "speed": speed
    })