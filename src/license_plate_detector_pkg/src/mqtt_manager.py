#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import os
import json
import ssl

# Get the path to the script directory
script_dir = os.path.dirname(os.path.realpath(__file__))
# Construct the absolute path to the certificate
cert_path = os.path.abspath(os.path.join(script_dir, "../certs", "certificate.crt"))

# MQTT Configuration
MQTT_BROKER = "mosquitto.ikt.fh-dortmund.de"
MQTT_PORT = 8883
MQTT_USERNAME = "iot-client"
MQTT_PASSWORD = "crYsARBqeT2R"
CA_CERT = cert_path

# Verify the existence of the certificate file
if not os.path.exists(CA_CERT):
    print(f"Certificate file not found at {CA_CERT}")
    exit(1)
else:
    print(f"Using certificate at: {CA_CERT}")
    # Read and print the certificate content for debugging
    try:
        with open(CA_CERT, 'r') as cert_file:
            cert_content = cert_file.read()
            print("Certificate content:")
            print(cert_content)
    except Exception as e:
        print(f"Failed to read the certificate file: {e}")
        exit(1)

# MQTT Client Setup
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.tls_set(
    ca_certs=CA_CERT,
    certfile=None,
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,
    tls_version=ssl.PROTOCOL_TLSv1_2
)
client.reconnect_delay_set(min_delay=1, max_delay=120)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Successfully connected to the MQTT broker.")
    else:
        print(f"Connection to MQTT broker failed with code {rc}.")

# Function to publish data to MQTT broker in JSON format
def publish_to_mqtt(automobile_id, data):
    try:
        mqtt_topic = f"/json/5jggokgpepnvsb2uv4s40d59kb/{automobile_id}/license_plate"

        # Convert data into a JSON object
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

        # Publish JSON string to the MQTT topic
        client.publish(mqtt_topic, json_string)
        print(f"Published to MQTT topic '{mqtt_topic}': {json_string}")
    except Exception as e:
        print(f"Failed to publish to MQTT: {e}")

# Callback function when a new message is received on 'license_plate_data' topic
def license_plate_callback(msg):
    try:
        # Print the received message data
        print(f"Received license plate data: {msg.data}")

        # Extract automobile_id from the message to use in the topic
        data_parts = msg.data.split(',')
        automobile_id = data_parts[0]

        # Publish the data to the MQTT topic as JSON
        publish_to_mqtt(automobile_id, msg.data)
    except IndexError:
        print("Received data format is incorrect. Expected format: 'automobile_id,date_time,number_plate,latitude,longitude,altitude'")

def listener():
    rospy.init_node('license_plate_mqtt_publisher', anonymous=True)
    rospy.Subscriber('license_plate_data', String, license_plate_callback)

    # Notify that the subscription is active
    print("Started listening to 'license_plate_data' topic...")

    # Connect to MQTT broker
    client.on_connect = on_connect
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        client.loop_stop()
        client.disconnect()
        print("ROS node stopped.")
