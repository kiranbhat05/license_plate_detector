import rospy
from sensor_msgs.msg import Image, NavSatFix, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os
from collections import deque, Counter
from datetime import datetime
import random
import math
import tf  # Import for quaternion to Euler conversion
# Import the openalpr module
script_dir = os.path.dirname(os.path.realpath(__file__))
lib_path = os.path.join(script_dir, '../lib')
if lib_path not in sys.path:
    sys.path.insert(0, lib_path)

import torch

from openalpr import Alpr
import re

import json
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend

class EncryptionManager:
    def __init__(self, public_key_file):
        """Initialize the encryption manager with RSA public key."""
        self.public_key = self.load_public_key(public_key_file)

    def load_public_key(self, filename):
        """Load RSA public key from a file."""
        with open(filename, "rb") as key_file:
            public_key = serialization.load_pem_public_key(
                key_file.read(),
                backend=default_backend()
            )
        return public_key

    def encrypt_data(self, data):
        """Encrypt serialized metadata using the RSA public key."""
        encrypted_data = self.public_key.encrypt(
            data.encode(),
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
        return encrypted_data


class Metadata:
    """Class to hold metadata information like time, date, number plate, GNSS data, and automobile ID."""
    def __init__(self, number_plate, gnss_data, automobile_id):
        self.automobile_id = automobile_id  # automobile ID
        self.date_time = datetime.now().isoformat()
        self.number_plate = number_plate
        self.gnss_data = gnss_data  # GNSS data
        

    def __str__(self):
        return f"AutomobileID: {self.automobile_id}, DateTime: {self.date_time}, Plate: {self.number_plate}, GNSS: {self.gnss_data}"

class YoloLicensePlateDetector:
    def __init__(self, confidence_threshold=0.7):
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
        results = self.model(frame)
        plates = []
        
        if results.xyxy[0].shape[0] > 0:
            for detection in results.xyxy[0]:
                conf = detection[4].item()
                if conf >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, detection[:4].tolist())
                    plates.append((x1, y1, x2, y2, conf))
        
        return plates

class LicensePlateDetector:
    def __init__(self, topic_name, gps_topic, orientation_topic, use_yolo):
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
        self.prev_gnss_data = None
        self.orientation = None  # Store orientation data from the /yaw topic
        self.use_yolo = use_yolo  # Store the parameter value

        self.automobile_id = random.randint(1000, 9999)

        # Initialize the encryption manager
        public_key_file = os.path.join(script_dir, "public_key.pem")
        self.encryption_manager = EncryptionManager(public_key_file)

        self.license_plate_pub = rospy.Publisher('license_plate_data', String, queue_size=10)

        if self.use_yolo:
            self.yolo_detector = YoloLicensePlateDetector()

        rospy.Subscriber(self.topic_name, Image, self.image_callback)
        rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
        rospy.Subscriber(orientation_topic, Imu, self.yaw_callback)

    def yaw_callback(self, data):
        """Callback function for the '/yaw' topic to get orientation and convert to direction."""
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # Yaw is the third element of the Euler angles (heading direction)
        yaw_degrees = math.degrees(yaw)

        # Normalize yaw_degrees to be between 0 and 360
        if yaw_degrees < 0:
            yaw_degrees += 360

        # Determine the orientation based on the yaw degrees
        if 337.5 <= yaw_degrees or yaw_degrees < 22.5:
            self.orientation = "north"
        elif 22.5 <= yaw_degrees < 67.5:
            self.orientation = "northeast"
        elif 67.5 <= yaw_degrees < 112.5:
            self.orientation = "east"
        elif 112.5 <= yaw_degrees < 157.5:
            self.orientation = "southeast"
        elif 157.5 <= yaw_degrees < 202.5:
            self.orientation = "south"
        elif 202.5 <= yaw_degrees < 247.5:
            self.orientation = "southwest"
        elif 247.5 <= yaw_degrees < 292.5:
            self.orientation = "west"
        elif 292.5 <= yaw_degrees < 337.5:
            self.orientation = "northwest"

        # Print yaw angle and interpreted orientation
        # print(f"Yaw angle: {yaw_degrees:.2f} degrees, Orientation: {self.orientation}")

    def gps_callback(self, data):
        """Callback function to update GNSS data and adjust based on direction."""
        current_gnss_data = {
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude
        }

        # Initialize previous GNSS data if it doesn't exist
        if self.prev_gnss_data is None:
            self.prev_gnss_data = current_gnss_data
            self.adjusted_gnss_data = current_gnss_data
            return

        # Adjust GNSS data based on topic_name and orientation
        if self.orientation == "east":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 2, 0)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -2, 0)
        
        elif self.orientation == "west":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -2, 0)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 2, 0)
        
        elif self.orientation == "north":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 0, -2)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 0, 2)
        
        elif self.orientation == "south":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 0, 2)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 0, -2)
        
        elif self.orientation == "northeast":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 1.5, 1.5)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -1.5, -1.5)
        
        elif self.orientation == "northwest":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -1.5, 1.5)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 1.5, -1.5)
        
        elif self.orientation == "southeast":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 1.5, -1.5)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -1.5, 1.5)
        
        elif self.orientation == "southwest":
            if self.topic_name == "processed_left_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, -1.5, -1.5)
            elif self.topic_name == "processed_right_frame":
                self.adjusted_gnss_data = self.adjust_coordinates(current_gnss_data, 1.5, 1.5)

        # Update the previous GNSS data for the next callback
        self.prev_gnss_data = current_gnss_data
        # print(f"Topic name: {self.topic_name}, Orientation: {self.orientation} topic: {self.topic_name}")
        # print(f"Adjusted GNSS Data: Lat: {self.adjusted_gnss_data['latitude']}, Lon: {self.adjusted_gnss_data['longitude']}, Alt: {self.adjusted_gnss_data['altitude']}")



    def adjust_coordinates(self, gnss_data, lateral_shift_meters, longitudinal_shift_meters):
        """Adjust latitude and longitude by a given number of meters."""
        
        # Earth's radius in meters (mean radius)
        earth_radius = 6378137  # meters

        # Latitude adjustment (longitudinal shift in meters)
        delta_latitude = longitudinal_shift_meters / 111320

        # Longitude adjustment (lateral shift in meters), accounting for latitude
        delta_longitude = lateral_shift_meters / (111320 * math.cos(math.radians(gnss_data['latitude'])))

        # Adjust the GNSS data with new latitude and longitude
        adjusted_gnss_data = {
            "latitude": gnss_data['latitude'] + delta_latitude,
            "longitude": gnss_data['longitude'] + delta_longitude,
            "altitude": gnss_data['altitude']  # No adjustment for altitude
        }

        return adjusted_gnss_data


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(f"CvBridge Error: {e}")
            return

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
            if count > 4:
                if self.prev_most_frequent_plate != most_frequent_plate:
                    self.check_plate_deep_history(most_frequent_plate)

    def check_plate_deep_history(self, shallow_history_plate):
        if self.adjusted_gnss_data is None:
            print("GNSS data is not available. Skipping storage.")
            return

        if shallow_history_plate not in self.plate_deep_history_rb:
            self.plate_deep_history_rb.append(shallow_history_plate)
            # Encrypt the numberplate using the EncryptionManager
            encrypted_numberplate = self.encryption_manager.encrypt_data(shallow_history_plate)

            metadata = Metadata(encrypted_numberplate.hex(), self.adjusted_gnss_data, self.automobile_id)
            print(f"Storing metadata in ring buffer: {metadata}")
            self.publish_license_plate_data(metadata)

    def publish_license_plate_data(self, metadata):
        data_string = f"{metadata.automobile_id},{metadata.date_time},{metadata.number_plate},{metadata.gnss_data['latitude']},{metadata.gnss_data['longitude']},{metadata.gnss_data['altitude']}"
        self.license_plate_pub.publish(data_string)
        #print(f"{self.topic_name} Published license plate data: {data_string}")


if __name__ == '__main__':
    rospy.init_node('license_plate_detector', anonymous=True)

    topic_name = rospy.get_param('~topic_name', 'video_frames')
    gps_topic = rospy.get_param('~gps_topic', '/gps/fix')
    orientation_topic = rospy.get_param('~orientation_topic', '/yaw')  # Using /yaw topic for orientation
    use_yolo = rospy.get_param('~use_yolo', True)

    detector = LicensePlateDetector(topic_name, gps_topic, orientation_topic, use_yolo)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()