# parrot_node.py
import rclpy
import time
import threading

from rclpy.node import Node
from parrot_msgs.msg import ServoMotorMsg, ServoMotorStatus
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
        self.state_pubs = {}



        self.get_logger().info("Performing servo setup check.")
        try:
            self.check_servos()
            self.get_logger().info("Servo check passed.")
        except Exception as e:
            self.get_logger().error(f"Servo setup failed: {e}")

         # Subscriptions for movements
        for name in servo.servos.keys():
            self.create_subscription(ServoMotorMsg, f"/servo/{name}", self._servo_cb, 10)
            self.create_subscription(String, f"/servo_method/{name}", self.method_callback, 10)

            self.state_pubs[name] = self.create_publisher(ServoMotorStatus, f"/servo/status/{name}", 10)
        

        # High level commands
        # self.create_subscription(BehaviourCommand, "/behaviour_cmd", self.behaviour_callback, 10)
        self.create_subscription(WingsCommand, "/wings_cmd", self.wings_callback, 10)

        self._locks = {t: threading.Lock() for t in ["head_rotate", "head_tilt", "left_wing", "right_wing"]}

        # default profile per target (override per call if you want)
        self._default_method = {
            "head_rotate": "Ease-In-Out",
            "head_tilt": "Ease-In-Out",
            "left_wing": "Ease-In-Out",
            "right_wing": "Ease-In-Out",
        }


    def _publish_status(self, target, commanded, method, speed, status, saturated=False, fault=False, message=""):
        msg = ServoMotorStatus()
        msg.target = target
        msg.commanded_deg = float(commanded)
        msg.measured_deg = float(commanded)  # no feedback yet
        msg.method = method
        msg.speed = float(speed)
        msg.status = status
        msg.saturated = saturated
        msg.fault = fault
        msg.message = message
        msg.stamp = self.get_clock().now().to_msg()
        self.state_pubs[target].publish(msg)


    def check_servos(self):
        self.get_logger().info("Checking all servos in parallel...")

        # --- Step 1: publish start + move to test positions ---
        test_config = {
            name: {"angle": servo.test_angles.get(name, 90), "method": "instant", "speed": 1.0}
            for name in servo.servos.keys()
        }

        servo.move_selected_servos(test_config)
        time.sleep(1)

        # --- Step 2: publish done + move back to initial positions ---
        reset_config = {
            name: {"angle": servo.init_angles.get(name, 90), "method": "instant", "speed": 1.0}
            for name in servo.servos.keys()
        }

        servo.move_selected_servos(reset_config)
        time.sleep(1)

        self.get_logger().info("Servo check complete.")

        servo.cleanup()
            
        
        
    def _servo_cb(self, msg: ServoMotorMsg):
        self.get_logger().info(f"Moving {msg.target} to {msg.position} with speed {msg.speed} using method {msg.method}")

        if msg.target not in servo.servos:
            self.get_logger().warn(f"Unknown target: {msg.target}")
            return

        # Publish "start"
        self._publish_status(msg.target, msg.position, msg.method, msg.speed, ServoMotorStatus.MOVING, message="start")

        # This already spawns a thread internally (parallel=True by default)
        servo.move_servo(msg.target, msg.position, method=msg.method, speed=msg.speed, parallel=True)

        # Immediately publish "done" — since we don't have feedback yet,
        # this just acknowledges the command was issued.
        self._publish_status(msg.target, msg.position, msg.method, msg.speed, ServoMotorStatus.IDLE, message="command sent")

        servo.cleanup()

    # def behaviour_callback(self, msg: BehaviourCommand):
    #     btype = msg.behaviour_type.lower()
    #     self.get_logger().info(f"[Behaviour] {btype} with amp={msg.amplitude}, speed={msg.speed}, reps={msg.repetitions}, target={msg.target}")

    #     if btype == "agree":
    #         self._do_agree(msg.amplitude, msg.speed)
    #     elif btype == "disagree":
    #         self._do_disagree(msg.amplitude, msg.speed)
    #     elif btype == "wave":
    #         self._do_wave(msg.target, msg.repetitions, msg.amplitude, msg.speed)
    #     elif btype == "lookaround":
    #         self._do_lookaround(msg.speed)
    #     else:
    #         self.get_logger().warn(f"Unknown behaviour type: {btype}")

    def wings_callback(self, msg: WingsCommand):
        self.get_logger().info(
            f"[Wings] Flapping wings L={msg.left_position}, R={msg.right_position}, "
            f"method={msg.method}, speed={msg.speed}, reps={msg.repetitions}"
        )

        for i in range(msg.repetitions):
            # --- Normal flap ---

            self.get_logger().info(f"[Wings] Repetition {i+1}: Normal")
            
            # Publish "start" statuses
            self._publish_status("left_wing", msg.left_position, msg.method, msg.speed,
                                ServoMotorStatus.MOVING, message="start")
            self._publish_status("right_wing", msg.right_position, msg.method, msg.speed,
                                ServoMotorStatus.MOVING, message="start")

            servo.move_selected_servos({
                "left_wing": {"angle": msg.left_position, "method": msg.method, "speed": msg.speed},
                "right_wing": {"angle": msg.right_position, "method": msg.method, "speed": msg.speed},
            })
            time.sleep(0.1)

            # --- Flipped flap ---
            self.get_logger().info(f"[Wings] Repetition {i+1}: Flipped")
            servo.move_selected_servos({
                "left_wing": {"angle": msg.right_position, "method": msg.method, "speed": msg.speed},
                "right_wing": {"angle": msg.left_position, "method": msg.method, "speed": msg.speed},
            })
            time.sleep(0.1)

            # Publish "done" statuses
            self._publish_status("left_wing", msg.left_position, msg.method, msg.speed,
                                ServoMotorStatus.IDLE, message="done")
            self._publish_status("right_wing", msg.right_position, msg.method, msg.speed,
                                ServoMotorStatus.IDLE, message="done")



    def method_callback(self, msg, source):
        self.get_logger().info(f"Received method '{msg.data}' on topic '{source}'")
        target = source.split('/')[-1]

        if target in self.servo_methods:
            self.servo_methods[target] = msg.data
            self.get_logger().info(f"Method for {target} set to {msg.data}")
        else:
            self.get_logger().warn(f"Unknown servo target in method topic: {target}")

    def _do_agree(self, amplitude: float, speed: float):
        """Nod head up–down and lift wings once (agreement)."""
        center = 90
        up = center - amplitude
        down = center + amplitude

        # Nod head (tilt)
        servo.move_servo("head_tilt", down, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("head_tilt", up, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("head_tilt", center, method="ease", speed=speed)

        # Small wing lift
        servo.move_servo("left_wing", 45, method="ease", speed=speed)
        servo.move_servo("right_wing", 135, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("left_wing", 170, method="ease", speed=speed)
        servo.move_servo("right_wing", 5, method="ease", speed=speed)

        servo.cleanup()


    def _do_disagree(self, amplitude: float, speed: float):
        """Shake head left–right and flap wings (disagreement)."""
        center = 90
        left = center - amplitude
        right = center + amplitude

        # Shake head
        servo.move_servo("head_rotate", left, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("head_rotate", right, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("head_rotate", center, method="ease", speed=speed)

        # Emphasize with wings
        servo.move_servo("left_wing", 45, method="ease", speed=speed)
        servo.move_servo("right_wing", 135, method="ease", speed=speed)
        time.sleep(0.5)
        servo.move_servo("left_wing", 170, method="ease", speed=speed)
        servo.move_servo("right_wing", 5, method="ease", speed=speed)

        servo.cleanup()


    def _do_wave(self, wing: str, repetitions: int, amplitude: float, speed: float):
        """Wave one wing up and down like saying hello."""
        if wing not in ["left", "right"]:
            self.get_logger().warn(f"Invalid wing: {wing}")
            return

        servo_name = f"{wing}_wing"
        if wing == "left":
            start = 170   # default down
            up = start - amplitude
            down = start
        else:
            start = 5    # default down
            up = start + amplitude
            down = start

        for i in range(repetitions):
            servo.move_servo(servo_name, up, method="ease", speed=speed)
            time.sleep(0.4)
            servo.move_servo(servo_name, down, method="ease", speed=speed)
            time.sleep(0.4)

        servo.cleanup()


    def _do_lookaround(self, speed: float):
        """Rotate head left–right–center (scanning)."""
        servo.move_servo("head_rotate", 30, method="ease", speed=speed)
        time.sleep(0.6)
        servo.move_servo("head_rotate", 150, method="ease", speed=speed)
        time.sleep(0.6)
        servo.move_servo("head_rotate", 90, method="ease", speed=speed)

        servo.cleanup()


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


