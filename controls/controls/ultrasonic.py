import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import lgpio
import time

# Constants
TIMER_PERIOD = 0.5  # seconds
MIN_DISTANCE = 0.1  # cm
MAX_DISTANCE = 20.0 # cm
SOUND_SPEED = 34300 # cm/s
TRIG_PIN = 20       # BCM pin number
ECHO_PIN = 21       # BCM pin number
STOP = 0

class UltrasonicNode(Node):
    def __init__(self, chip):
        super().__init__("ultrasonic_node")
        self.publisher_ = self.create_publisher(Int16, "controls/l298n", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.detect_obstacle)
        self.get_logger().info("Ultrasonic sensor initialized")
        self.chip = chip

    def detect_obstacle(self):
        # Send trigger pulse
        lgpio.gpio_write(self.chip, TRIG_PIN, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, TRIG_PIN, 0)

        # Measure echo time
        start = time.time()
        while lgpio.gpio_read(self.chip, ECHO_PIN) == 0:
            start = time.time()
        while lgpio.gpio_read(self.chip, ECHO_PIN) == 1:
            end = time.time()

        duration = end - start
        distance = (duration * SOUND_SPEED) / 2

        if MIN_DISTANCE <= distance <= MAX_DISTANCE:
            msg = Int16()
            msg.data = STOP
            self.publisher_.publish(msg)
            self.get_logger().info(f"Obstacle detected at {distance:.1f} cm")
            
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    # Open GPIO chip (0 = default /dev/gpiochip0)
    chip = lgpio.gpiochip_open(4)

    # Set up pins
    lgpio.gpio_claim_output(chip, TRIG_PIN)
    lgpio.gpio_claim_input(chip, ECHO_PIN)

    # ROS2 setup
    rclpy.init(args=args)
    node = UltrasonicNode(chip)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ultrasonic sensor stopped")
        lgpio.gpio_write(chip, TRIG_PIN, 0)
        lgpio.gpio_write(chip, ECHO_PIN, 0)

    # Cleanup
    node.destroy_node()
    lgpio.gpiochip_close(chip)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
