#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix  # Import for GPS data
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os
import importlib
from collections import Counter, deque
from datetime import datetime  # Import for date and time

# Import the openalpr module
script_dir = os.path.dirname(os.path.realpath(__file__))
lib_path = os.path.join(script_dir, '../lib')
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)

from openalpr import Alpr

import re

class Metadata:
    """Class to hold metadata information like time, date, number plate, and GNSS data."""
    def __init__(self, number_plate, gnss_data):
        self.date_time = datetime.now()
        self.number_plate = number_plate
        self.gnss_data = gnss_data  # Use real GNSS data

    def __str__(self):
        return f"DateTime: {self.date_time}, Plate: {self.number_plate}, GNSS: {self.gnss_data}"

class LicensePlateDetector:
    def __init__(self, topic_name, gps_topic):
        # Initialize ALPR
        runtime_data_path = os.path.join(script_dir, "../lib/runtime_data")
        self.alpr = Alpr("eu", "/etc/openalpr/alprd.conf", runtime_data_path)
        if not self.alpr.is_loaded():
            print("Error loading OpenALPR")
            rospy.signal_shutdown("OpenALPR failed to load")

        self.alpr.set_top_n(20)
        self.alpr.set_default_region("md")
        self.bridge = CvBridge()
        self.plate_history = []
        self.plate_deep_history_rb = deque(maxlen=5)  # Ring buffer of size 5
        self.topic_name = topic_name
        self.prev_most_frequent_plate = 'Do123456'
        self.gnss_data = None  # To store GNSS data

        rospy.Subscriber(self.topic_name, Image, self.image_callback)
        rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)  # Subscriber for GPS data

    def gps_callback(self, data):
        """Callback function to update GNSS data."""
        self.gnss_data = {
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude
        }

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
            return

        # Process the frame
        self.process_frame(frame)

    def process_frame(self, frame):
        self.process_plate(frame, 75) # to test decreased the confidence level

        cv2.imshow(f"{self.topic_name}", frame)
        cv2.waitKey(1)

    def process_plate(self, frame, detection_confidence=85):
        results_alpr = self.alpr.recognize_ndarray(frame)
        if not results_alpr['results']:
            return

        for result in results_alpr['results']:
            plate = result['plate']
            confidence = result['confidence']

            if self.is_invalid_plate(plate):
                continue
            
            if confidence > detection_confidence:
                self.check_plate_shallow_history(plate)

    def is_invalid_plate(self, plate):
        if len(set(plate)) == 1:
            return True
        if plate in ["IIIIIIII", "IIIIIII", "IIIIII", "OOOO", "OOOOO", "IIIIII1", "JIIIIIII"]:
            return True
        if len(plate) < 6 or len(plate) > 9:
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
            if count > 2:
                if self.prev_most_frequent_plate != most_frequent_plate:
                    print(f"Detected license plate: {most_frequent_plate} from topic: {self.topic_name}")
                    self.check_plate_deep_history(most_frequent_plate)
                    self.prev_most_frequent_plate = most_frequent_plate
                self.plate_history.clear()

    def check_plate_deep_history(self, shallow_history_plate):
        """Process the detected plate, storing it in the ring buffer if not already present."""
        if shallow_history_plate not in self.plate_deep_history_rb:
            if self.gnss_data is None:
                print("GNSS data is not available. Skipping storage.")
                return
            metadata = Metadata(shallow_history_plate, self.gnss_data)
            self.plate_deep_history_rb.append(metadata)
            print(f"Storing metadata in ring buffer: {metadata}")
        else:
            print(f"Plate {shallow_history_plate} is already in the ring buffer.")

if __name__ == '__main__':
    rospy.init_node('license_plate_detector', anonymous=True)

    # Arguments to specify which topics to listen to
    topic_name = rospy.get_param('~topic_name', 'video_frames')
    gps_topic = rospy.get_param('~gps_topic', '/gps/fix')  # Added parameter for GPS topic
    detector = LicensePlateDetector(topic_name, gps_topic)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
