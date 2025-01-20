#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor:
    """
    A class to process images for license plate detection.
    Initializes ROS node, publishers, and subscribers for image processing.
    """

    def __init__(self):
        """
        Initializes the ImageProcessor class and sets up ROS node, publishers, and subscribers.

        Parameters:
        None

        Returns:
        None
        """

        # Initialize ROS node with a unique name
        rospy.init_node('license_plate_detector_preprocessor', anonymous=True)

        # Set up publishers for the processed left and right image frames
        self.pub_left = rospy.Publisher('processed_left_frame', Image, queue_size=1)
        self.pub_right = rospy.Publisher('processed_right_frame', Image, queue_size=1)

        # Initialize CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the /image_raw topic to receive incoming image data
        self.image_sub = rospy.Subscriber(
            '/image_raw', Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )

        # Set the percentage of the image width used to divide the image into left and right parts
        self.left_width_percent = rospy.get_param('~left_width_percent', 50) / 100.0  # Default is 50%

        # Define the cropping percentages for the left and right images
        self.left_image_crops = {
            'left': rospy.get_param('~left_image_left_side_crop', 10) / 100.0,
            'right': rospy.get_param('~left_image_right_side_crop', 10) / 100.0,
            'top': rospy.get_param('~left_image_top_crop', 20) / 100.0,
            'bottom': rospy.get_param('~left_image_bottom_crop', 15) / 100.0
        }

        self.right_image_crops = {
            'left': rospy.get_param('~right_image_left_side_crop', 10) / 100.0,
            'right': rospy.get_param('~right_image_right_side_crop', 10) / 100.0,
            'top': rospy.get_param('~right_image_top_crop', 20) / 100.0,
            'bottom': rospy.get_param('~right_image_bottom_crop', 15) / 100.0
        }

    def image_callback(self, msg):
        """
        Callback function to process the incoming image message.

        Parameters:
        msg (sensor_msgs.msg.Image): The incoming image message.

        Returns:
        None
        """
        # Convert the ROS Image message to a cv2 image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Get the dimensions of the image
        height, width, _ = frame.shape

        # Calculate the mid_x based on the left_width_percent
        mid_x = int(width * self.left_width_percent)

        # Split the frame into left and right parts and apply cropping
        left_frame_cropped = self.crop_image(frame[:, :mid_x], self.left_image_crops, height, mid_x)
        right_frame_cropped = self.crop_image(frame[:, mid_x:], self.right_image_crops, height, width-mid_x)

        # Convert the cv2 images back to ROS Image messages
        left_image_message = self.bridge.cv2_to_imgmsg(left_frame_cropped, encoding="bgr8")
        right_image_message = self.bridge.cv2_to_imgmsg(right_frame_cropped, encoding="bgr8")

        # Publish the left and right frames separately to the new topics
        self.pub_left.publish(left_image_message)
        self.pub_right.publish(right_image_message)

    def crop_image(self, frame, crops, height, width):
        """
        Crops a given image frame based on specified percentages of the frame's width and height.

        Parameters:
        frame (numpy.ndarray): The original image frame to be cropped.
        crops (dict): A dictionary containing the percentages for left, right, top, and bottom cropping.
        height (int): The height of the original frame.
        width (int): The width of the original frame.

        Returns:
        numpy.ndarray: The cropped image frame.
        """

        # Calculate crop values based on the percentages and frame dimensions
        crop_x_left = int(width * crops['left'])
        crop_x_right = int(width * crops['right'])
        crop_y_top = int(height * crops['top'])
        crop_y_bottom = int(height * crops['bottom'])

        # Apply cropping to the frame using the calculated crop values
        return frame[
            crop_y_top:height-crop_y_bottom,
            crop_x_left:width-crop_x_right
        ]

    def spin(self):
        """
        Keeps the ROS node running and prevents it from exiting. This function should be called
        after setting up all the publishers and subscribers to allow the node to continuously
        listen for incoming messages and respond to them.

        Parameters:
        None

        Returns:
        None
        """
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the ImageProcessor class
        processor = ImageProcessor()
        # Start processing the images
        processor.spin()
    except rospy.ROSInterruptException:
        pass
