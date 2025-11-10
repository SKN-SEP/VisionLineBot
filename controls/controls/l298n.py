import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import lgpio
import time

# Constants
US_TOPIC = "/ultrasonic_obstacle_detection"
MIN_PWM_DUTY_CYCLE = 30  # %
MAX_PWM_DUTY_CYCLE = 100 # %
PWM = 1000               #kHz

ENA_PIN = 18 # BCM pin number
IN1_PIN = 15 # BCM pin number
IN2_PIN = 14 # BCM pin number
IN3_PIN = 2  # BCM pin number
IN4_PIN = 3  # BCM pin number
ENB_PIN = 4  # BCM pin number

class L298nNode(Node):

    def __init__(self, chip):
        super().__init__("l298n_node")
        self.subscriber = self.create_subscription(Bool, US_TOPIC, self.update_flag, 10)
        self.is_obstacle = True

        self.get_logger().info("L298n motor module initialized")
        self.chip = chip

    def update_flag(self, msg):
        self.is_obstacle = msg.data
        self.get_logger().info(f"is_obstacle updated: {self.is_obstacle}")

    def control_motors(self, msg):
        self.get_logger().info(f"L298n motor module has message {msg}")
        # To be continued...
    
    def spin_left_motor_forward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.chip, IN1_PIN, 0)
        lgpio.gpio_write(self.chip, IN2_PIN, 1)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.chip, ENA_PIN, PWM, speed)
    
    def spin_left_motor_backward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.chip, IN1_PIN, 1)
        lgpio.gpio_write(self.chip, IN2_PIN, 0)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.chip, ENA_PIN, PWM, speed)

    def stop_left_motor(self):
        lgpio.gpio_write(self.chip, IN1_PIN, 0)
        lgpio.gpio_write(self.chip, IN2_PIN, 0)
        lgpio.tx_pwm(self.chip, ENA_PIN, PWM, 0)
    
    def spin_right_motor_forward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.chip, IN3_PIN, 1)
        lgpio.gpio_write(self.chip, IN4_PIN, 0)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.chip, ENB_PIN, PWM, speed)
    
    def spin_right_motor_backward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.chip, IN3_PIN, 0)
        lgpio.gpio_write(self.chip, IN4_PIN, 1)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.chip, ENB_PIN, PWM, speed)

    def stop_right_motor(self):
        lgpio.gpio_write(self.chip, IN3_PIN, 0)
        lgpio.gpio_write(self.chip, IN4_PIN, 0)
        lgpio.tx_pwm(self.chip, ENB_PIN, PWM, 0)

    def destroy_node(self):
        # Stop l298n module
        self.get_logger().info("L298n motor module stopped")
        lgpio.gpio_write(self.chip, ENA_PIN, 0)
        lgpio.gpio_write(self.chip, IN1_PIN, 0)
        lgpio.gpio_write(self.chip, IN2_PIN, 0)
        lgpio.gpio_write(self.chip, IN3_PIN, 0)
        lgpio.gpio_write(self.chip, IN4_PIN, 0)
        lgpio.gpio_write(self.chip, ENB_PIN, 0)

        super().destroy_node()


def main(args=None):
    # Open GPIO chip (4 = default /dev/gpiochip4)
    chip = lgpio.gpiochip_open(4)

    # Set up pins
    lgpio.gpio_claim_output(chip, ENA_PIN)
    lgpio.gpio_claim_output(chip, IN1_PIN)
    lgpio.gpio_claim_output(chip, IN2_PIN)
    lgpio.gpio_claim_output(chip, IN3_PIN)
    lgpio.gpio_claim_output(chip, IN4_PIN)
    lgpio.gpio_claim_output(chip, ENB_PIN)

    # ROS2 setup
    rclpy.init(args=args)
    node = L298nNode(chip)
    node.stop_left_motor()
    node.stop_right_motor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Cleanup
        node.destroy_node()
        lgpio.gpiochip_close(chip)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
