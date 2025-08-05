# wings_node.py
import rclpy
import time
from rclpy.node import Node
from parrot_msgs.msg import ServoMotorMsg

import parrot_robot.servo_driver as servo


class WingsNode(Node):
    def __init__(self):
        super().__init__('wings_node')
        self.get_logger().info("[WingsNode] Initialized")

        # Set initial positions
        for name, angle in servo.last_angles.items():
            if name in ["left_wing", "right_wing"] and name in servo.servos:
                servo.servos[name].angle = angle
                self.get_logger().info(f"[WingsNode] Set {name} to {angle}° at startup")

        self.get_logger().info("[WingsNode] Performing servo setup check.")
        try:
            self.check_servos()
            servo.cleanup()
            self.get_logger().info("[WingsNode] Servo check passed.")
        except Exception as e:
            self.get_logger().error(f"[WingsNode] Servo setup failed: {e}")

        self.create_subscription(ServoMotorMsg, '/servo/left_wing', self.move_callback, 1)
        self.create_subscription(ServoMotorMsg, '/servo/right_wing', self.move_callback, 1)

    def check_servos(self):
        # Move to known positions to confirm operation
        for target in ["left_wing", "right_wing"]:
            servo.move_servo(target, servo.last_angles[target], method="Instant", speed=1.0)
            time.sleep(1)
            servo.move_servo(target, 90, method="Instant", speed=1.0)
            time.sleep(1)
            servo.move_servo(target, servo.last_angles[target], method="Instant", speed=1.0)
            time.sleep(1)

    def move_callback(self, msg):
        self.get_logger().info(f"[WingsNode] Moving {msg.target} to position {msg.position} with speed {msg.speed}")
        if msg.target == "left_wing":
            self.handle_left_wing(msg.position, msg.speed)
        elif msg.target == "right_wing":
            self.handle_right_wing(msg.position, msg.speed)
        else:
            self.get_logger().warn(f"[WingsNode] Unknown target: {msg.target}")

        self.get_logger().info("[WingsNode] Cleaning up servos after move.")
        servo.cleanup()

    def handle_left_wing(self, position, speed):
        servo.move_servo("left_wing", position, speed=speed)

    def handle_right_wing(self, position, speed):
        servo.move_servo("right_wing", position, speed=speed)


def main(args=None):
    rclpy.init(args=args)
    node = WingsNode()
    rclpy.spin(node)
    servo.cleanup()
    node.destroy_node()
    rclpy.shutdown()