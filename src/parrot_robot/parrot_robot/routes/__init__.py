from .interactions_routes import interactions_bp
from .servo_routes import servo_bp
from .context import set_ros_node



def register_routes(app, node):
    
    app.register_blueprint(interactions_bp)
    app.register_blueprint(servo_bp)
    app.logger.info("[Routes] Registered all routes with Flask app")
