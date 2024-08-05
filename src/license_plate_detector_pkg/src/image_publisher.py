#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_publisher(video_path):
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    cap = cv2.VideoCapture(video_path)
    bridge = CvBridge()

    if not cap.isOpened():
        rospy.logerr(f"Error: Could not open video '{video_path}'")
        return

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Publish the frame
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(image_message)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        video_path = "/home/kiran/Ros_Repo/r1_example/src/robo_pckg/src/videos/R0010009_st.MP4"
        image_publisher(video_path)
    except rospy.ROSInterruptException:
        pass
