# http_server_node.py
import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS

from parrot_msgs.msg import NodCommand, WingsCommand, SoundCommand
from parrot_msgs.msg import ServoMotorMsg # type: ignore
import threading

from .routes import register_routes
from .routes.context import set_ros_node

# Setting up the flask http server with blueprint routes
app = Flask(__name__, static_folder='static')
CORS(app)

ros_node = None

class HTTPBridge(Node):
   
    def __init__(self):
        super().__init__('http_server_node')
        self.servo_pub = self.create_publisher(ServoMotorMsg, '/servo_cmd', 10)
        


    def publish_servo(self, target, position, speed):
        msg = ServoMotorMsg()
        msg.target = target
        msg.position = position
        msg.speed = speed
        topic = f"/servo/{target}"
        self.get_logger().info(f"[HTTP] ServoCommand to {topic}: pos={position}, speed={speed}")
        
        pub = self.create_publisher(ServoMotorMsg, topic, 10)
        pub.publish(msg)

@app.route('/test', methods=['GET'])
def test_route():
    if ros_node:
        return jsonify({"status": "OK", "node_name": ros_node.get_name()})
    else:
        return jsonify({"status": "ROS not ready"}), 503

def ros_spin():
    rclpy.spin(ros_node)

def main():
    global ros_node
    rclpy.init()
    ros_node = HTTPBridge()
    set_ros_node(ros_node)
    register_routes(app, ros_node)

    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(debug=True, host="0.0.0.0", port=5000)  # Open server on port 5000

if __name__ == "__main__":
    main()
