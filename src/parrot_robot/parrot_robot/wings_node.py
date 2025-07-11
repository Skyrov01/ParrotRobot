# wings_node.py
import rclpy
from rclpy.node import Node
from parrot_msgs.msg import ServoMotorMsg

import parrot_robot.servo_driver as servo

class WingsNode(Node):
    def __init__(self):
        super().__init__('wings_node')
        self.create_subscription(ServoMotorMsg, '/servo/left_wing', self.move_callback, 10)
        self.create_subscription(ServoMotorMsg, '/servo/right_wing', self.move_callback, 10)

    def move_callback(self, msg):
        self.get_logger().info(f"[WingsNode] Moving {msg.target} to position {msg.position} with speed {msg.speed}")
        # servo_driver.move_servo(msg.target, msg.position, speed=msg.speed)

def main(args=None):
    rclpy.init(args=args)
    node = WingsNode()
    rclpy.spin(node)
    servo.cleanup()
    node.destroy_node()
    rclpy.shutdown()
