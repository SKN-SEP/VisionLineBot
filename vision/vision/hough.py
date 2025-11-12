# Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import numpy as np
import cv2 as cv
import uuid
import os

# Constants
CAMERA_ID = 0
TIMER_DURATION_S = 2.5
SPEED_TOPIC = "/cv_speed"
MOVE_TOPIC = "/cv_move"
IMG_DIR = "/home/pi/VisionLineBotSrc/VisionLineBot/vision/images"

class HoughLineFollowerAlgorithm(Node):

    def __init__(self):
        super().__init__("hough_node")

        # Initialize ROS2
        self.__speed_publisher = self.create_publisher(Int16, SPEED_TOPIC, 10)
        self.__move_publisher = self.create_publisher(Int16, MOVE_TOPIC, 10)
        self.__timer = self.create_timer(TIMER_DURATION_S, self.followLine)

        # Initialize OpenCV
        self.__camera = cv.VideoCapture(CAMERA_ID)
        self.__frame = None
        self.__ret = None 
        assert self.__camera != None 
    
    def captureFrame(self):
        # Take picture
        self.__ret, self.__frame = self.__camera.read()

        # Perform image processing
        if self.__ret:
            pass
    
    def saveFrame(self):
        # Generate file name
        filename = f"{IMG_DIR}/{uuid.uuid4().__str__()}.jpg"

        # Save file 
        if self.__ret:
            # Path safety check        
            if not os.path.exists(IMG_DIR):
                os.mkdir(IMG_DIR)
            
            cv.imwrite(filename, self.__frame)

    def followLine(self):
        # Capture and save image
        self.captureFrame()
        self.saveFrame()

        # TO BE CONTINUED

    def destroy_node(self):
        self.__camera.release()
        super().destroy_node()

# Entry point
def main(args=None):
    # ROS2 setup
    rclpy.init(args=args)
    node = HoughLineFollowerAlgorithm()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()