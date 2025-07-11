from .interactions_routes import interactions_bp
from .servo_routes import servo_bp, ros_node



ros_node = None  # Shared globally inside routes package

def register_routes(app):

    app.register_blueprint(interactions_bp)
    app.register_blueprint(servo_bp)

