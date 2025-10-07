# sound_node.py
import rclpy
from rclpy.node import Node
from parrot_msgs.msg import SoundCommand
import os
import subprocess

# Try pygame first
try:
    import pygame
    pygame.mixer.init()
    USE_PYGAME = True
except Exception as e:
    print(f"[WARN] pygame not available, falling back to aplay: {e}")
    USE_PYGAME = False

SOUND_DIR = os.path.expanduser("~/ParrotRobot/src/sounds")

class SoundNode(Node):
    def __init__(self):
        super().__init__("sound_node")
        self.create_subscription(SoundCommand, "/sound_cmd", self.callback, 10)
        self.get_logger().info("SoundNode ready and waiting for /sound_cmd")

    def callback(self, msg):
        sound_file = os.path.join(SOUND_DIR, f"{msg.sound_name}.wav")
        if not os.path.exists(sound_file):
            self.get_logger().error(f"Sound file not found: {sound_file}")
            return

        self.get_logger().info(f"Playing sound: {msg.sound_name}")

        if USE_PYGAME:
            try:
                pygame.mixer.music.load(sound_file)
                pygame.mixer.music.play()
            except Exception as e:
                self.get_logger().error(f"pygame error, using aplay fallback: {e}")
                self._play_with_aplay(sound_file)
        else:
            self._play_with_aplay(sound_file)

    def _play_with_aplay(self, sound_file):
        """Fallback: call ALSA's aplay utility."""
        try:
            subprocess.Popen(["aplay", sound_file])
            self.get_logger().info(f"Playing via aplay: {sound_file}")
        except Exception as e:
            self.get_logger().error(f"aplay failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()