#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class MapPlotter:
    def __init__(self):
        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('License Plate Locations')
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        
        # List to store data points
        self.data_points = []

        rospy.Subscriber('license_plate_data', String, self.license_plate_callback)

        # Start a timer to update the plot
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)

    def license_plate_callback(self, msg):
        """Callback function to handle incoming license plate data."""
        data = msg.data.split(',')
        number_plate = data[0]
        latitude = float(data[1])
        longitude = float(data[2])
        altitude = float(data[3])

        # Store the data point
        self.data_points.append((latitude, longitude, number_plate, altitude))
        print(f"Received data point: {number_plate} at ({latitude}, {longitude}), Altitude: {altitude}")

    def update_plot(self, frame):
        """Update the scatter plot with new data points."""
        # Clear previous plots
        self.ax.clear()
        self.ax.set_title('License Plate Locations')
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')

        for latitude, longitude, number_plate, altitude in self.data_points:
            self.ax.scatter(longitude, latitude, label=f"Plate: {number_plate}, Altitude: {altitude}", s=50, alpha=0.7)
            self.ax.annotate(number_plate, (longitude, latitude), textcoords="offset points", xytext=(5, 5), ha='center')

        plt.draw()

if __name__ == '__main__':
    rospy.init_node('map_plotter', anonymous=True)
    plotter = MapPlotter()
    
    plt.show()  # Show the plot

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
