import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16
import lgpio
import time

# Constants
US_TOPIC = "/ultrasonic_obstacle_detection"
MOTORS_TOPIC = "/motors"
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
        self.__us_subscriber = self.create_subscription(Bool, US_TOPIC, self.update_flag, 10)
        self.__motors_subscriber = self.create_subscription(Int16, MOTORS_TOPIC, self.control_motors, 10)
        self.__is_obstacle = True

        self.get_logger().info("L298n motor module initialized")
        self.__chip = chip

    def update_flag(self, msg):
        self.__is_obstacle = msg.data
        self.get_logger().info(f"is_obstacle updated: {self.__is_obstacle}")

    def control_motors(self, msg):
        self.get_logger().info(f"L298n motor module has message {msg}")
        
        # Unpack velocity
        u = msg.data
        vL = MIN_PWM_DUTY_CYCLE + u/2
        vR = MIN_PWM_DUTY_CYCLE - u/2

        if self.__is_obstacle:
            self.get_logger().info(f"Motors stopped!")
            self.stop_left_motor()
            self.stop_right_motor()
            return

        # Set left motor
        if vL >= 0:
            self.spin_left_motor_forward(vL)
        else:
            vl *= -1
            self.spin_left_motor_backward(vL)
        self.get_logger().info(f"Left motor speed updated!")

        # Set right motor
        if vR >= 0:
            self.spin_right_motor_forward(vR)
        else:
            vR *= -1
            self.spin_right_motor_backward(vR)
        self.get_logger().info(f"Right motor speed updated!")

    
    def spin_left_motor_forward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.__chip, IN1_PIN, 0)
        lgpio.gpio_write(self.__chip, IN2_PIN, 1)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.__chip, ENA_PIN, PWM, speed)
    
    def spin_left_motor_backward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.__chip, IN1_PIN, 1)
        lgpio.gpio_write(self.__chip, IN2_PIN, 0)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.__chip, ENA_PIN, PWM, speed)

    def stop_left_motor(self):
        lgpio.gpio_write(self.__chip, IN1_PIN, 0)
        lgpio.gpio_write(self.__chip, IN2_PIN, 0)
        lgpio.tx_pwm(self.__chip, ENA_PIN, PWM, 0)
    
    def spin_right_motor_forward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.__chip, IN3_PIN, 1)
        lgpio.gpio_write(self.__chip, IN4_PIN, 0)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.__chip, ENB_PIN, PWM, speed)
    
    def spin_right_motor_backward(self, speed=MIN_PWM_DUTY_CYCLE):
        lgpio.gpio_write(self.__chip, IN3_PIN, 0)
        lgpio.gpio_write(self.__chip, IN4_PIN, 1)

        speed = max(MIN_PWM_DUTY_CYCLE, min(speed, MAX_PWM_DUTY_CYCLE))
        lgpio.tx_pwm(self.__chip, ENB_PIN, PWM, speed)

    def stop_right_motor(self):
        lgpio.gpio_write(self.__chip, IN3_PIN, 0)
        lgpio.gpio_write(self.__chip, IN4_PIN, 0)
        lgpio.tx_pwm(self.__chip, ENB_PIN, PWM, 0)

    def destroy_node(self):
        # Stop l298n module
        self.get_logger().info("L298n motor module stopped")
        lgpio.gpio_write(self.__chip, ENA_PIN, 0)
        lgpio.gpio_write(self.__chip, IN1_PIN, 0)
        lgpio.gpio_write(self.__chip, IN2_PIN, 0)
        lgpio.gpio_write(self.__chip, IN3_PIN, 0)
        lgpio.gpio_write(self.__chip, IN4_PIN, 0)
        lgpio.gpio_write(self.__chip, ENB_PIN, 0)

        super().destroy_node()


def main(args=None):
    # Open GPIO chip (0 = default /dev/gpiochip0)
    chip = lgpio.gpiochip_open(0)

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
