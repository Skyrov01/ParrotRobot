# head_node.py
import rclpy
import time
from rclpy.node import Node
from parrot_msgs.msg import ServoMotorMsg

import parrot_robot.servo_driver as servo



class HeadNode(Node):
    def __init__(self):
        super().__init__('head_node')
        self.get_logger().info("[HeadNode] Initialized")

        # Set initial positions
        for name, angle in servo.last_angles.items():
            if name in servo.servos and name in ["head_rotate", "head_tilt"]:
                servo.servos[name].angle = angle
                self.get_logger().info(f"[HeadNode] Set {name} to {angle}° at startup")

        self.get_logger().info("[HeadNode] Performing servo setup check.")
        try:
            self.check_servos()
            servo.cleanup()
            self.get_logger().info("[HeadNode] Servo check passed.")

        except Exception as e:
            self.get_logger().error(f"[HeadNode] Servo setup failed: {e}")


        self.create_subscription(ServoMotorMsg, '/servo/head_rotate', self.move_callback, 1)
        self.create_subscription(ServoMotorMsg, '/servo/head_tilt', self.move_callback, 1)

    def check_servos(self):
        # Move to known positions to confirm operation
        servo.move_servo("head_rotate", 0, method="Instant", speed=1.0)
        time.sleep(1)
        servo.move_servo("head_rotate", 90, method="Instant", speed=1.0)
        time.sleep(1)
        servo.move_servo("head_rotate", 180, method="Instant", speed=1.0)
        time.sleep(1)
        servo.move_servo("head_rotate", 90, method="Instant", speed=1.0)


    def move_callback(self, msg):
        self.get_logger().info(f"[HeadNode] Moving {msg.target} to position {msg.position} with speed {msg.speed}")
        # Call the servo driver to move the head
        if msg.target == "head_rotate":
            self.handle_head_rotate(msg.position, msg.speed)
        elif msg.target == "head_tilt":
            self.handle_head_tilt(msg.position, msg.speed)
        else:
            self.get_logger().warn(f"[HeadNode] Unknown target: {msg.target}")
        
        # Cleanup servos after use
        self.get_logger().info("[HeadNode] Cleaning up servos after move.")
        servo.cleanup()

    def handle_head_rotate(self, position, speed):
        servo.move_servo("head_rotate", position, speed=speed)

    def handle_head_tilt(self, position, speed):
        servo.move_servo("head_tilt", position, speed=speed)


def main(args=None):
    rclpy.init(args=args)
    node = HeadNode()
    rclpy.spin(node)
    servo.cleanup()
    node.destroy_node()
    rclpy.shutdown()
