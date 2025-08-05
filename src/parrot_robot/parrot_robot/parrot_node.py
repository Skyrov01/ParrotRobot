# parrot_node.py
import rclpy
import time
import threading

from rclpy.node import Node
from parrot_msgs.msg import ServoMotorMsg
from std_msgs.msg import String 


from parrot_msgs.msg import WingsCommand 

import parrot_robot.servo_driver as servo


class ParrotNode(Node):
    def __init__(self):
        super().__init__('parrot_node')
        self.get_logger().info("Initialized")

        self.servo_methods = {name: "Instant" for name in servo.servos.keys()}

        # Set initial positions
        servo.init_servos()


        self.get_logger().info("Performing servo setup check.")
        try:
            self.check_servos()
            self.get_logger().info("Servo check passed.")
        except Exception as e:
            self.get_logger().error(f"Servo setup failed: {e}")

         # Subscriptions for movements
        for name in servo.servos.keys():
            self.create_subscription(ServoMotorMsg, f"/servo/{name}", self.move_callback, 10)
            self.create_subscription(String, f"/servo_method/{name}", self.method_callback, 10)

        self.create_subscription(WingsCommand, "/wings_cmd", self.wings_callback, 10)


    def check_servos(self):
        for target in servo.servos.keys():
            self.get_logger().info(f"Checking servo: {target}")
            # Move to known positions to confirm operation
            angle = servo.test_angles.get(target, 90)
            initial_position = servo.last_angles.get(target, 90)

            servo.move_servo(target, angle, method="instant", speed=1.0)
            time.sleep(1)
            servo.move_servo(target, initial_position, method="instant", speed=1.0)
            time.sleep(1)
            servo.cleanup()

    def move_callback(self, msg):
        self.get_logger().info(f"Moving {msg.target} to {msg.position} with speed {msg.speed} using method {msg.method}")

        if msg.target not in servo.servos:
            self.get_logger().warn(f"Unknown target: {msg.target}")
            return

        def do_move():
            servo.move_servo(msg.target, msg.position, method=msg.method, speed=msg.speed)
            servo.cleanup()

        # Run in background so ROS keeps going
        threading.Thread(target=do_move).start()

    def wings_callback(self, msg):
        self.get_logger().info(f"[Wings] Flapping wings L={msg.left_position}, R={msg.right_position}, method={msg.method}, speed={msg.speed}, reps={msg.repetitions}")

        def move_pair(left, right):
            servo.move_servo("left_wing", left, method=msg.method, speed=msg.speed)
            servo.move_servo("right_wing", right, method=msg.method, speed=msg.speed)

        def move_pair_flipped(left, right):
            servo.move_servo("left_wing", right, method=msg.method, speed=msg.speed)
            servo.move_servo("right_wing", left, method=msg.method, speed=msg.speed)

        for i in range(msg.repetitions):
            self.get_logger().info(f"[Wings] Repetition {i+1}: Normal")
            t1 = threading.Thread(target=servo.move_servo, args=("left_wing", msg.left_position), kwargs={"method": msg.method, "speed": msg.speed})
            t2 = threading.Thread(target=servo.move_servo, args=("right_wing", msg.right_position), kwargs={"method": msg.method, "speed": msg.speed})
            t1.start()
            t2.start()
            t1.join()
            t2.join()
            time.sleep(0.1)

            self.get_logger().info(f"[Wings] Repetition {i+1}: Flipped")
            t3 = threading.Thread(target=servo.move_servo, args=("left_wing", msg.right_position), kwargs={"method": msg.method, "speed": msg.speed})
            t4 = threading.Thread(target=servo.move_servo, args=("right_wing", msg.left_position), kwargs={"method": msg.method, "speed": msg.speed})
            t3.start()
            t4.start()
            t3.join()
            t4.join()
            time.sleep(0.1)

        servo.cleanup()


    def method_callback(self, msg, source):
        self.get_logger().info(f"Received method '{msg.data}' on topic '{source}'")
        target = source.split('/')[-1]

        if target in self.servo_methods:
            self.servo_methods[target] = msg.data
            self.get_logger().info(f"Method for {target} set to {msg.data}")
        else:
            self.get_logger().warn(f"Unknown servo target in method topic: {target}")


def main(args=None):
    rclpy.init(args=args)
    node = ParrotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        servo.cleanup()
        node.destroy_node()
        rclpy.shutdown()


