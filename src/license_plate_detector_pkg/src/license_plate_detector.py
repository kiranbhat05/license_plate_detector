#!/usr/bin/env python3

import sys
import os

# Add the path to the lib/Openalpr directory where openalpr.py is located
script_dir = os.path.dirname(os.path.realpath(__file__))
lib_path = os.path.join(script_dir, '../lib')
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)

# Now import Alpr from openalpr.py
from openalpr import Alpr

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import re
from collections import Counter

class LicensePlateDetector:
    def __init__(self):
        # Get the runtime data path relative to this script
        runtime_data_path = os.path.join(script_dir, "../lib/runtime_data")

        self.alpr = Alpr("eu", "/etc/openalpr/alprd.conf", runtime_data_path)
        if not self.alpr.is_loaded():
            rospy.logerr("Error loading OpenALPR")
            rospy.signal_shutdown("OpenALPR failed to load")

        self.alpr.set_top_n(20)
        self.alpr.set_default_region("md")
        self.bridge = CvBridge()
        self.plate_history = []

        rospy.Subscriber("video_frames", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Process the frame
        self.process_frame(frame)

    def process_frame(self, frame):
        height, width, _ = frame.shape
        start_x = width // 2
        cropped_frame = frame[:, start_x:width]
        self.process_plate(cropped_frame)

        small_frame = cv2.resize(cropped_frame, (1600, 800))
        cv2.imshow("Cropped Frame", small_frame)
        cv2.waitKey(1)

    def process_plate(self, frame, detection_confidence=85):
        results_alpr = self.alpr.recognize_ndarray(frame)
        if not results_alpr['results']:
            return

        for result in results_alpr['results']:
            plate = result['plate']
            confidence = result['confidence']

            if self.is_invalid_plate(plate):
                rospy.loginfo(f"Discarded invalid plate: {plate}")
                continue

            if confidence > detection_confidence:
                rospy.loginfo(f"Detected license plate: {plate} with confidence {confidence}%")
                self.check_plate_shallow_history(plate)

    def is_invalid_plate(self, plate):
        if len(set(plate)) == 1:
            return True
        if plate in ["IIIIIIII", "IIIIIII", "IIIIII", "OOOO", "OOOOO", "IIIIII1", "JIIIIIII"]:
            return True
        if len(plate) < 6 or len(plate) > 8:
            return True
        if plate.isdigit() or plate.isalpha():
            return True
        if re.match(r"^(.)\1+$", plate):
            return True
        if re.match(r"^(.)\1*$", plate[1:]):
            return True
        return False

    def check_plate_shallow_history(self, plate, history_size=6):
        self.plate_history.append(plate)
        if len(self.plate_history) > history_size:
            self.plate_history.pop(0)
        
        if len(self.plate_history) == history_size:
            plate_counts = Counter(self.plate_history)
            most_frequent_plate, count = plate_counts.most_common(1)[0]
            if count > 3:
                rospy.loginfo(f"Detected license plate: {most_frequent_plate}")
                self.plate_history.clear()

if __name__ == '__main__':
    rospy.init_node('license_plate_detector', anonymous=True)
    detector = LicensePlateDetector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
