from flask import Blueprint, request, jsonify
from .context import ros_node  




helper_bp = Blueprint("helper", __name__)

ros_node = None  


# Test route to confirm the server is running
@helper_bp.route('/ping', methods=['GET'])
def ping():
    return jsonify({"status": "ok", "message": "Parrot HTTP Bridge is alive!"})