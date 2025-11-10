import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio
import time

# Constants
TOPIC = "/ultrasonic_obstacle_detection"
TIMER_PERIOD = 0.2  # seconds
MIN_DISTANCE = 0.1  # cm
MAX_DISTANCE = 20.0 # cm
SOUND_SPEED = 34300 # cm/s
TRIG_PIN = 20       # BCM pin number
ECHO_PIN = 21       # BCM pin number

class UltrasonicNode(Node):
    
    def __init__(self, chip):
        super().__init__("ultrasonic_node")
        self.publisher = self.create_publisher(Bool, TOPIC, 10)
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

        # Compute distance
        duration = end - start
        distance = (duration * SOUND_SPEED) / 2

        # Publish message
        msg = Bool()
        msg.data = (MIN_DISTANCE <= distance <= MAX_DISTANCE)
        self.publisher.publish(msg)
        self.get_logger().info(f"Obstacle detected at {distance:.1f} cm")
            
    def destroy_node(self):
        # Stop ultrasonic sensor
        self.get_logger().info("Ultrasonic sensor stopped")
        lgpio.gpio_write(self.chip, TRIG_PIN, 0)
        super().destroy_node()

def main(args=None):
    # Open GPIO chip (4 = default /dev/gpiochip4)
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
        # Cleanup
        node.destroy_node()
        lgpio.gpiochip_close(chip)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
