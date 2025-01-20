#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

# Function to convert data to JSON format
def convert_to_json(data):
    """
    Convert a comma-separated string of license plate data to JSON format.

    This function takes a string containing license plate information and converts
    it into a JSON-formatted string. The input string should contain six fields
    separated by commas: automobile ID, date and time, number plate, latitude,
    longitude, and altitude.

    Parameters:
    data (str): A comma-separated string containing license plate information.
                Expected format: 'automobile_id,date_time,number_plate,latitude,longitude,altitude'

    Returns:
    str or None: A JSON-formatted string containing the license plate data if successful,
                 or None if the input format is incorrect.

    Raises:
    IndexError: If the input string does not contain the expected number of fields.
    ValueError: If the latitude, longitude, or altitude cannot be converted to float.
    """
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
    """
    Callback function for processing license plate data messages.

    This function is called when a new message is received on the 'license_plate_data' topic.
    It converts the received data to JSON format and publishes it to the 'number_plate_json' topic.

    Parameters:
    msg (std_msgs.msg.String): The received message containing license plate data.
                               Expected to be a comma-separated string.

    Returns:
    None

    Side effects:
    - Publishes JSON-formatted license plate data to the 'number_plate_json' topic.
    - Prints success or error messages to the console.
    """
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
    """
    This function initializes a ROS node, subscribes to the 'license_plate_data' topic,
    and publishes the received data in JSON format to the 'number_plate_json' topic.

    Parameters:
    None

    Returns:
    None

    Side effects:
    - Initializes a ROS node named 'license_plate_in_json' with anonymous=True.
    - Subscribes to the 'license_plate_data' topic with a callback function 'license_plate_callback'.
    - Creates a publisher for the 'number_plate_json' topic with a queue size of 10.
    - Prints a message indicating that the subscription to 'license_plate_data' topic is active.
    - Spins the ROS node to keep it running.
    """
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
