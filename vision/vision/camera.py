# Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import uuid
import os

# Constants
TIMER_DURATION_S =  0.1
LINE_CAMERA_ID = 0
LINE_TOPIC = "/camera/line_images"
IMG_DIR = "/home/pi/VisionLineBotSrc/VisionLineBot/vision/images"
IMG_ENCODING = "bgr8"

class LineCameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        # Initialize ROS2
        self.__publisher = self.create_publisher(Image, LINE_TOPIC, 10)
        self.__timer = self.create_timer(TIMER_DURATION_S, self.captureFrame)
        self.__bridge = CvBridge()

        # Initialize OpenCV
        self.__camera = cv.VideoCapture(LINE_CAMERA_ID)
        self.__frame = None
        self.__ret = None 
        assert self.__camera != None 
        self.get_logger().info("Camera has been initialized")
    
    def captureFrame(self):
        # Take picture
        self.__ret, self.__frame = self.__camera.read()

        # Upload image to the topic
        if self.__ret:
            img = self.__bridge.cv2_to_imgmsg(self.__frame, IMG_ENCODING)
            self.__publisher.publish(img)
            self.get_logger().info(f"Image published to the topic: {LINE_TOPIC}")
    
    def saveFrame(self):
        # Generate file name
        filename = f"{IMG_DIR}/{uuid.uuid4().__str__()}.jpg"

        # Save file 
        if self.__ret:
            # Path safety check        
            if not os.path.exists(IMG_DIR):
                os.mkdir(IMG_DIR)
            
            cv.imwrite(filename, self.__frame)

    def destroy_node(self):
        self.__camera.release()
        self.get_logger().info("Camera has stopped")
        super().destroy_node()

# Entry point
def main(args=None):
    # ROS2 setup
    rclpy.init(args=args)
    node = LineCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()