#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import plotly.graph_objects as go
import os

class MapPlotter:
    def __init__(self):
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

        # Subscribe to the license_plate_data topic
        rospy.Subscriber('license_plate_data', String, self.license_plate_callback)

        # Register the shutdown hook
        rospy.on_shutdown(self.save_map_on_shutdown)

    def license_plate_callback(self, msg):
        """Callback function to handle incoming license plate data."""
        data = msg.data.split(',')
        number_plate = data[0]
        latitude = float(data[1])
        longitude = float(data[2])
        altitude = float(data[3])

        # Store the data in a dictionary with the number plate as the key
        self.plate_data[number_plate] = {
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude
        }

        print(f"Received data point: {number_plate} at ({latitude}, {longitude}), Altitude: {altitude}")

    def update_map_plot(self):
        """Update the Plotly map with all collected data points at once."""
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
        """Save the map when ROS node is shutting down."""
        self.save_map()

    def save_map(self, file_name="license_plate_map.html"):
        """Save the map data to a file."""
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
