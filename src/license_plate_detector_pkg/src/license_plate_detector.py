#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os
import importlib
from collections import Counter, deque
from datetime import datetime

# Import the openalpr module
script_dir = os.path.dirname(os.path.realpath(__file__))
lib_path = os.path.join(script_dir, '../lib')
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)

import torch  # Import torch for YOLO model
import math
import torch

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

class YoloLicensePlateDetector:
    def __init__(self, confidence_threshold=0.7):
        """Initialize the YOLO model for license plate detection."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, '../data/alpr_weights.pt')  # Relative path to the model weights
        
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
            print("YOLO model loaded successfully.")
        except Exception as e:
            print(f"Error loading YOLOv5 model: {e}")
            exit()
        self.confidence_threshold = confidence_threshold

    def detect_license_plate(self, frame):
        """Detect license plate regions in the image."""
        results = self.model(frame)
        plates = []
        
        # Check for detections and filter by confidence threshold
        if results.xyxy[0].shape[0] > 0:
            for detection in results.xyxy[0]:
                conf = detection[4].item()
                if conf >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, detection[:4].tolist())
                    plates.append((x1, y1, x2, y2, conf))
        
        return plates

class LicensePlateDetector:
    def __init__(self, topic_name, gps_topic, use_yolo):
        # Initialize ALPR
        script_dir = os.path.dirname(os.path.realpath(__file__))
        runtime_data_path = os.path.join(script_dir, "../lib/runtime_data")
        self.alpr = Alpr("eu", "/etc/openalpr/alprd.conf", runtime_data_path)
        if not self.alpr.is_loaded():
            print("Error loading OpenALPR")
            rospy.signal_shutdown("OpenALPR failed to load")

        self.alpr.set_top_n(20)
        self.alpr.set_default_region("md")
        self.bridge = CvBridge()
        self.plate_history = []
        self.plate_deep_history_rb = deque(maxlen=5)
        self.topic_name = topic_name
        self.prev_most_frequent_plate = 'Do123456'
        self.gnss_data = None
        self.adjusted_gnss_data = None
        self.prev_gnss_data = None  # Store previous GNSS data
        self.use_yolo = use_yolo  # Store the parameter value

        # Initialize a publisher for license plate and GNSS data
        self.license_plate_pub = rospy.Publisher('license_plate_data', String, queue_size=10)

        if self.use_yolo:
            self.yolo_detector = YoloLicensePlateDetector()  # Initialize YOLO detector

        rospy.Subscriber(self.topic_name, Image, self.image_callback)
        rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)

    def gps_callback(self, data):
        """Callback function to update GNSS data."""
        current_gnss_data = {
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude
        }

        if self.prev_gnss_data is None:
            # First data point; no previous data to compare with
            self.prev_gnss_data = current_gnss_data
            self.adjusted_gnss_data = current_gnss_data
            return

        # Compute the differences in latitude and longitude
        delta_latitude = current_gnss_data['latitude'] - self.prev_gnss_data['latitude']
        delta_longitude = current_gnss_data['longitude'] - self.prev_gnss_data['longitude']

        # Determine the direction of movement
        if abs(delta_latitude) > abs(delta_longitude):
            # Predominant movement is north-south
            self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 0, 2 if delta_latitude > 0 else -2)
        elif abs(delta_longitude) > abs(delta_latitude):
            # Predominant movement is east-west
            self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 2 if delta_longitude > 0 else -2, 0)
        else:
            # Significant movement in both directions
            self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 
                                                             2 if delta_longitude > 0 else -2, 
                                                             2 if delta_latitude > 0 else -2)

        # Update the previous GNSS data for the next comparison
        self.prev_gnss_data = current_gnss_data

    def adjust_coordinates(self, gnss_data, lateral_shift_meters, longitudinal_shift_meters):
        """Adjust latitude and longitude by a given number of meters."""
        # Convert meters to degrees
        delta_latitude = longitudinal_shift_meters / 111320  # Adjust latitude based on movement
        delta_longitude = lateral_shift_meters / (111320 * math.cos(math.radians(gnss_data['latitude'])))  # Adjust longitude

        adjusted_gnss_data = {
            "latitude": gnss_data['latitude'] + delta_latitude,
            "longitude": gnss_data['longitude'] + delta_longitude,
            "altitude": gnss_data['altitude']
        }

        return adjusted_gnss_data

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
            return

        # Process the frame
        self.process_frame(frame)

    def process_frame(self, frame):
        if self.use_yolo:
            license_plate_regions = self.yolo_detector.detect_license_plate(frame)
            
            if not license_plate_regions:
                return

            for (x1, y1, x2, y2, conf) in license_plate_regions:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                confidence_text = f'{conf * 100:.2f}%'
                cv2.putText(frame, confidence_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                cropped_number_plate = frame[y1:y2, x1:x2]
                self.process_plate(cropped_number_plate, 75)
        else:
            # Directly pass the entire frame to ALPR
            self.process_plate(frame, 75)

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

    def check_plate_shallow_history(self, plate, history_size=8):
        self.plate_history.append(plate)
        if len(self.plate_history) > history_size:
            self.plate_history.pop(0)
        
        if len(self.plate_history) == history_size:
            plate_counts = Counter(self.plate_history)
            most_frequent_plate, count = plate_counts.most_common(1)[0]
            if count > 3:
                if self.prev_most_frequent_plate != most_frequent_plate:
                    self.check_plate_deep_history(most_frequent_plate)

    def check_plate_deep_history(self, shallow_history_plate):
        """Process the detected plate, storing it in the ring buffer if not already present."""
        if self.adjusted_gnss_data is None:
            print("GNSS data is not available. Skipping storage.")
            return

        # Only add metadata if it isn't already in the ring buffer
        if shallow_history_plate not in self.plate_deep_history_rb:
            self.plate_deep_history_rb.append(shallow_history_plate)
            metadata = Metadata(shallow_history_plate, self.adjusted_gnss_data)
            print(f"Storing metadata in ring buffer: {metadata}")

            # Publish the license plate data and GNSS coordinates
            self.publish_license_plate_data(metadata)

    def publish_license_plate_data(self, metadata):
        """Publish license plate data and GNSS coordinates."""
        data_string = f"{metadata.number_plate},{metadata.gnss_data['latitude']},{metadata.gnss_data['longitude']},{metadata.gnss_data['altitude']}"
        self.license_plate_pub.publish(data_string)
        print(f"{self.topic_name} Published license plate data: {data_string}")

if __name__ == '__main__':
    rospy.init_node('license_plate_detector', anonymous=True)

    # Get parameters from launch file
    topic_name = rospy.get_param('~topic_name', 'video_frames')
    gps_topic = rospy.get_param('~gps_topic', '/gps/fix')
    use_yolo = rospy.get_param('~use_yolo', True)  # Parameter to enable or bypass YOLO

    detector = LicensePlateDetector(topic_name, gps_topic, use_yolo)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
