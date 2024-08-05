#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        # Initialize ROS node with the new name
        rospy.init_node('license_plate_detector_preprocessor', anonymous=True)
        
        # Initialize publishers for left and right images with new topic names
        self.pub_left = rospy.Publisher('processed_left_frame', Image, queue_size=1)
        self.pub_right = rospy.Publisher('processed_right_frame', Image, queue_size=1)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the /image_raw topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)

        # Percentage to divide the image into left and right parts
        self.left_width_percent = rospy.get_param('~left_width_percent', 50) / 100.0  # Default is 50%
        
        # Cropping percentages for each side of the left and right images
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
        # Calculate crop values
        crop_x_left = int(width * crops['left'])
        crop_x_right = int(width * crops['right'])
        crop_y_top = int(height * crops['top'])
        crop_y_bottom = int(height * crops['bottom'])
        
        # Apply cropping
        return frame[
            crop_y_top:height-crop_y_bottom, 
            crop_x_left:width-crop_x_right
        ]

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the ImageProcessor class
        processor = ImageProcessor()
        # Start processing the images
        processor.spin()
    except rospy.ROSInterruptException:
        pass
