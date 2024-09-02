#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import folium
import os

class MapPlotter:
    def __init__(self):
        # Initialize the map
        self.map_file_path = "license_plate_map.html"
        self.map_center = [51.5065289834961, 7.4567108137787255]
        
        if os.path.exists(self.map_file_path):
            print(f"Loading existing map from {self.map_file_path}.")
            self.map = folium.Map(location=self.map_center, zoom_start=20)
        else:
            print(f"Creating a new map at {self.map_file_path}.")
            self.map = folium.Map(location=self.map_center, zoom_start=20)

        rospy.Subscriber('license_plate_data', String, self.license_plate_callback)

    def license_plate_callback(self, msg):
        """Callback function to handle incoming license plate data."""
        data = msg.data.split(',')
        number_plate = data[0]
        latitude = float(data[1])
        longitude = float(data[2])
        altitude = float(data[3])

        # Add a marker for the detected license plate
        folium.Marker(
            location=[latitude, longitude],
            popup=f"Plate: {number_plate}, Altitude: {altitude}",
            icon=folium.Icon(color='blue', icon='info-sign')
        ).add_to(self.map)

        # Save or update the map file
        self.map.save(self.map_file_path)
        print(f"Map updated with new marker. Saved to {self.map_file_path}.")

if __name__ == '__main__':
    rospy.init_node('map_plotter', anonymous=True)
    plotter = MapPlotter()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
