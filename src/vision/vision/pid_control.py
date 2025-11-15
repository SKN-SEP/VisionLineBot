# Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from vision.image_tools import detect_line

# Node constants
TIMER_DURATION_S =  0.1
LINE_CAMERA_ID = 0.05
LINE_TOPIC = "/camera/line_images"
MOTORS_TOPIC = "/motors"
IMG_ENCODING = "bgr8"

# PID constants
MIDDLE_OF_IMG = 320 # Setpoint
KP = 0.05 # Steers toward the line
KI = 0 # Eliminates drift/curve overset
KD = 0.01 # Reduces overshoot and oscilation

class PIDNode(Node):

    def __init__(self):
        super().__init__("pid_node")

        # Initialize ROS2
        self.__publisher = self.create_publisher(Int16, MOTORS_TOPIC, 10)
        self.__subscriber = self.create_subscription(Image, LINE_TOPIC, self.centerRobot, 10)
        self.__bridge = CvBridge()

        # Initialize variables
        self.__prev_error = 0
        self.__integral = 0

        self.get_logger().info("PID controller has been initialized")
    
    def centerRobot(self, msg, dt=TIMER_DURATION_S):
        # Unpack OpenCV image
        img = self.__bridge.imgmsg_to_cv2(msg, IMG_ENCODING)

        # Calculate error
        curr_error = MIDDLE_OF_IMG - detect_line(img)

        # Propotional term
        p_out = KP * curr_error 

        # Integral term
        self.__integral += curr_error * dt
        i_out = KI * self.__integral
        
        # Derivative term
        derivative = (curr_error - self.__prev_error) / dt
        d_out = KD * derivative

        # Compute total output
        output = p_out + i_out + d_out

        # Update previous error
        self.__prev_error = curr_error

        # Publish decision
        velocity_msg = Int16()
        velocity_msg.data = int(output)
        self.__publisher.publish(velocity_msg)
    
    def destroy_node(self):
        self.get_logger().info("PID controller has stopped")
        super().destroy_node()

# Entry point
def main(args=None):
    # ROS2 setup
    rclpy.init(args=args)
    node = PIDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()