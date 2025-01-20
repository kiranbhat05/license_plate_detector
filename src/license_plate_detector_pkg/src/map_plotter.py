#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import plotly.graph_objects as go
import os
import json
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.backends import default_backend

# Class to decrypt the license plate data
class DecryptionManager:
    def __init__(self, private_key_file):
        """Initialize the decryption manager with RSA private key."""
        self.private_key = self.load_private_key(private_key_file)

    def load_private_key(self, filename):
        """Load RSA private key from a file."""
        with open(filename, "rb") as key_file:
            private_key = serialization.load_pem_private_key(
                key_file.read(),
                password=None,
                backend=default_backend()
            )
        return private_key

    def decrypt_data(self, encrypted_data):
        """Decrypt the encrypted data using RSA private key."""
        decrypted_data = self.private_key.decrypt(
            encrypted_data,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
        return decrypted_data.decode()

class MapPlotter:
    def __init__(self):
        """
        Initializes the MapPlotter object.

        This function sets up the initial state of the MapPlotter object, including:
        - Initializing the map center and zoom level for Plotly.
        - Setting up an empty dictionary to store number plates and their respective coordinates and altitude.
        - Setting up the Plotly mapbox figure with the initial layout.
        - Loading the private key for decryption.
        - Subscribing to the license_plate_data topic.
        - Registering the shutdown hook to save the map when the ROS node is shutting down.
        """
        # Initialize the map center and zoom level for Plotly
        self.map_center = [51.5065289834961, 7.4567108137787255]
        self.zoom_level = 15

        # Dictionary to store number plates and their respective coordinates and altitude
        self.plate_data = {}

        # Set up the Plotly mapbox figure only once
        self.map_fig = go.Figure()

        # Set initial mapbox layout
        self.map_fig.update_layout(
            mapbox_style="open-street-map",
            mapbox=dict(
                center=dict(lat=self.map_center[0], lon=self.map_center[1]),
                zoom=self.zoom_level
            ),
            margin={"r": 0, "t": 0, "l": 0, "b": 0},
            showlegend=False
        )

        # Load the private key for decryption
        # Get the path to the script directory
        script_dir = os.path.dirname(os.path.realpath(__file__))
        private_key_file = os.path.join(script_dir, "../certs", "private_key.pem")
        self.decryption_manager = DecryptionManager(private_key_file)

        # Subscribe to the license_plate_data topic
        rospy.Subscriber('license_plate_data', String, self.license_plate_callback)

        # Register the shutdown hook
        rospy.on_shutdown(self.save_map_on_shutdown)

    def license_plate_callback(self, msg):
        """
        Callback function to handle incoming license plate data.

        Parameters:
        msg (std_msgs.msg.String): The ROS message containing the license plate data.

        The function does the following:
        1. Splits the data in the message by comma.
        2. Decrypts the number plate data using a private RSA key.
        3. Processes the remaining GNSS and other data.
        4. Stores the data in a dictionary with the decrypted number plate as the key.
        5. Prints a summary of the received data point.

        Returns:
        None
        """
        data = msg.data.split(',')

        # Decrypt the number plate data (third element, index 2)
        encrypted_number_plate_hex = data[2]  # The encrypted number plate is at index 2
        encrypted_number_plate = bytes.fromhex(encrypted_number_plate_hex)  # Convert hex to bytes

        # Decrypt the license plate using the DecryptionManager
        number_plate = self.decryption_manager.decrypt_data(encrypted_number_plate)

        # Process the remaining GNSS and other data
        automobile_id = int(data[0])  # Use square brackets to access list elements
        print(f"Automobile ID: {automobile_id}")
        date_time = data[1]  # No need to cast this to str since it's already a string
        print(f"Date/Time: {date_time}")
        latitude = float(data[3])  # Convert latitude to float
        longitude = float(data[4])  # Convert longitude to float
        altitude = float(data[5])  # Convert altitude to float

        # Store the data in a dictionary with the decrypted number plate as the key
        self.plate_data[number_plate] = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude
        }

        print(f"Received data point: {number_plate} at ({latitude}, {longitude}), Altitude: {altitude}")

    def update_map_plot(self):
        """
        Update the Plotly map with all collected data points at once.

        This function extracts latitudes, longitudes, and plate information from the stored data,
        clears any previous traces on the map, and adds all the stored data points to the Plotly map.
        Each data point is represented as a marker on the map, with its information displayed when hovered over.

        Parameters:
            None

        Returns:
            None
        """
        # Extract latitudes, longitudes, and plate info from the stored data
        latitudes = [data['latitude'] for data in self.plate_data.values()]
        longitudes = [data['longitude'] for data in self.plate_data.values()]
        plates_info = [f"Plate: {plate}, Altitude: {data['altitude']}" for plate, data in self.plate_data.items()]

        # Clear previous traces so we don't stack new traces on top of old ones
        self.map_fig.data = []

        # Add all stored data points to the Plotly map
        self.map_fig.add_trace(go.Scattermapbox(
            lat=latitudes,  # Use the entire list of latitudes
            lon=longitudes,  # Use the entire list of longitudes
            mode='markers',
            marker=go.scattermapbox.Marker(size=14, color='blue'),
            text=plates_info  # Show information for all plates
        ))

    def save_map_on_shutdown(self):
        """
        Save the map data to a file when the ROS node is shutting down.

        This method is called as a shutdown hook to ensure that the map data is saved
        to a file before the ROS node is terminated. It internally calls the `save_map`
        method to perform the actual saving operation.

        Parameters:
            None

        Returns:
            None
        """
        self.save_map()

    def save_map(self, file_name="license_plate_map.html"):
        """
        Save the map data to a file.

        This function updates the map with all collected data points and then
        saves the map data to a file in HTML format. The file name can be
        specified as an argument, with a default value of "license_plate_map.html".

        Parameters:
            file_name (str): The name of the file to save the map data to.
                Defaults to "license_plate_map.html".

        Returns:
            None: This function does not return a value. Instead, it saves the map data
                to a file and prints a message indicating the file name and location.
        """
        # Update the map with all collected points
        self.update_map_plot()

        # Save the map
        self.map_fig.write_html(file_name)
        print(f"Map saved to {file_name}. Location: {os.path.abspath(file_name)}")

if __name__ == '__main__':
    rospy.init_node('map_plotter', anonymous=True)
    plotter = MapPlotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # Map saving is now handled by the shutdown hook
