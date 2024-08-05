#!/usr/bin/env python3

import rospy
import csv
import rospkg
from sensor_msgs.msg import NavSatFix

def gnss_publisher():
    # Initialize the ROS node
    rospy.init_node('gnss_publisher', anonymous=True)
    
    # Create a ROS publisher
    pub = rospy.Publisher('gnss_data', NavSatFix, queue_size=10)
    
    # Set the publishing rate
    rate = rospy.Rate(1)  # 1 Hz

    # Get the path to the gnss_data.txt file
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('license_plate_detector_pkg')
    file_path = package_path + '/data/gnss_data.txt'

    # Open the GNSS data file
    with open(file_path, 'r') as file:
        # Skip lines until the header is found
        while True:
            line = file.readline()
            if line.startswith("PacketCounter"):
                # After finding the header line, reset the file pointer to this line
                file.seek(file.tell() - len(line))
                break

        reader = csv.DictReader(file)

        # Debugging: Print the actual headers found
        headers = reader.fieldnames
        rospy.loginfo(f"Headers found: {headers}")

        # Ensure that 'Latitude', 'Longitude', 'Altitude' are in the headers
        if 'Latitude' not in headers or 'Longitude' not in headers or 'Altitude' not in headers:
            rospy.logerr("The file does not contain the required headers: 'Latitude', 'Longitude', 'Altitude'")
            return

        for row in reader:
            # Skip rows with NaN values or missing keys
            try:
                if row['Latitude'] == 'NaN' or row['Longitude'] == 'NaN' or row['Altitude'] == 'NaN':
                    continue

                # Create a new NavSatFix message
                gnss_msg = NavSatFix()
                
                # Populate the message with data from the file
                gnss_msg.header.stamp = rospy.Time.now()
                gnss_msg.latitude = float(row['Latitude'].strip())
                gnss_msg.longitude = float(row['Longitude'].strip())
                gnss_msg.altitude = float(row['Altitude'].strip())
                
                # Publish the message
                pub.publish(gnss_msg)
                
            except KeyError as e:
                rospy.logwarn(f"Missing key in row: {e}, skipping row.")
            except ValueError as e:
                rospy.logwarn(f"Invalid data in row: {e}, skipping row.")
            
            # Sleep to maintain the loop rate
            rate.sleep()

if __name__ == '__main__':
    try:
        gnss_publisher()
    except rospy.ROSInterruptException:
        pass
