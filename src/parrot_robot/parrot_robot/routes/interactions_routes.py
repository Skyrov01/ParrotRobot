from flask import Flask, request, Response, jsonify, Blueprint
from .context import ros_node  
interactions_bp = Blueprint("interactions", __name__)


@interactions_bp.route('/nod/<speed>/<int:count>', methods=['POST'])
def nod_route(speed, count):
    
    # Here you could publish a NodCommand
    return jsonify({
        "status": "nod triggered",
        "speed": speed,
        "count": count
    })

