#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

# Function to convert data to JSON format
def convert_to_json(data):
    try:
        # Split the received data
        data_parts = data.split(',')
        json_data = {
            "automobile_id": data_parts[0],
            "date_time": data_parts[1],
            "number_plate": data_parts[2],
            "latitude": float(data_parts[3]),
            "longitude": float(data_parts[4]),
            "altitude": float(data_parts[5])
        }
        json_string = json.dumps(json_data)
        return json_string
    except IndexError:
        print("Received data format is incorrect. Expected format: 'automobile_id,date_time,number_plate,latitude,longitude,altitude'")
        return None

# Callback function when a new message is received on 'license_plate_data' topic
def license_plate_callback(msg):
    try:
        # Convert the data to JSON format
        json_string = convert_to_json(msg.data)

        # If conversion is successful, publish the JSON string to 'number_plate_json' topic
        if json_string:
            json_msg = String()
            json_msg.data = json_string
            json_publisher.publish(json_msg)
            print(f"Published JSON data to 'number_plate_json': {json_string}")
    except Exception as e:
        print(f"Failed to process message: {e}")

def listener():
    global json_publisher
    rospy.init_node('license_plate_in_json', anonymous=True)
    rospy.Subscriber('license_plate_data', String, license_plate_callback)
    
    # Publisher for the JSON formatted data
    json_publisher = rospy.Publisher('number_plate_json', String, queue_size=10)

    # Notify that the subscription is active
    print("Started listening to 'license_plate_data' topic...")

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("ROS node stopped.")
