# http_server_node.py
import rclpy
from rclpy.node import Node
from flask import Flask, request, jsonify
from flask_cors import CORS

from std_msgs.msg import String  # already included in parrot_node

from parrot_msgs.msg import NodCommand, WingsCommand, SoundCommand
from parrot_msgs.msg import ServoMotorMsg # type: ignore
import threading

from .routes import register_routes
from .routes.context import set_ros_node

from threading import Lock, Thread

# Setting up the flask http server with blueprint routes
app = Flask(__name__, static_folder='static')
CORS(app)

ros_node = None

class HTTPBridge(Node):
   
    def __init__(self):
        super().__init__('http_server_node')
        self.servo_pub = self.create_publisher(ServoMotorMsg, '/servo_cmd', 10)
        
        self.servo_locks = {
            "head_rotate": Lock(),
            "head_tilt": Lock(),
            "left_wing": Lock(),
            "right_wing": Lock()
        }

    def get_servo_status(self, target):
        lock = self.servo_locks.get(target)
        if lock is None:
            return None
        return lock.locked()    

    def publish_servo(self, target, position, speed, method):
        lock = self.servo_locks.get(target)
        if not lock:
            self.get_logger().warn(f"Unknown target: {target}")
            return

        def run():
            if not lock.acquire(blocking=False):
                self.get_logger().warn(f"{target} is busy.")
                return
            try:
                msg = ServoMotorMsg()
                msg.target = target
                msg.position = position
                msg.speed = speed
                msg.method = method
                topic = f"/servo/{target}"
                self.get_logger().info(f"[HTTP] ServoCommand to {topic}: pos={position}, speed={speed}, method={method}")
                pub = self.create_publisher(ServoMotorMsg, topic, 10)
                pub.publish(msg)
            finally:
                lock.release()

        Thread(target=run).start()

    def publish_wings(self, left, right, speed, method="Instant", repetitions=1):
        msg = WingsCommand()
        msg.left_position = left
        msg.right_position = right
        msg.speed = speed
        msg.method = method
        msg.repetitions = repetitions

        self.get_logger().info(f"[HTTP] WingsCommand: L={left}, R={right}, speed={speed}, method={method}, reps={repetitions}")
        pub = self.create_publisher(WingsCommand, "/wings_cmd", 10)
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
