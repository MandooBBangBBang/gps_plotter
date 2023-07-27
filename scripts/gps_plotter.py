#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.interpolate import interp1d
import folium
import mplleaflet
import webbrowser
from sensor_msgs.msg import NavSatFix
import rospy
import re
from datetime import datetime, timezone, timedelta

class GPSPlotter:
    def __init__(self):
        self.publisher = None
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 10))

    def format_altitude(self, altitude):
        return altitude / 1000

    def format_coordinate(self, coord):
        return coord / 1e7

    def linear_interpolation(self, x, y, t):
        x = np.asarray(x)
        y = np.asarray(y)
        t = np.asarray(t)

        x_indices = np.floor(t).astype(int)
        x_indices = np.clip(x_indices, 0, len(x) - 2)
        x_deltas = t - x_indices

        return y[x_indices] + x_deltas * (y[x_indices + 1] - y[x_indices])

    def plot_2D_graph(self, latitude, longitude):
        latitude = [self.format_coordinate(lat) for lat in latitude]
        longitude = [self.format_coordinate(lon) for lon in longitude]

        self.ax[0].clear()
        self.ax[0].plot(latitude, longitude, marker='o', linestyle='-')
        self.ax[0].set_xlabel('Latitude')
        self.ax[0].set_ylabel('Longitude')
        self.ax[0].scatter([latitude[0]], [longitude[0]], marker='x', s=100, color='red', label='Start')
        self.ax[0].scatter([latitude[-1]], [longitude[-1]], marker='x', s=100, color='red', label='End')
        self.ax[0].legend()
        plt.show()

    def plot_3D_graph(self, latitude, longitude, altitude):
        latitude = [self.format_coordinate(lat) for lat in latitude]
        longitude = [self.format_coordinate(lon) for lon in longitude]
        altitude = [self.format_altitude(alt) for alt in altitude]

        self.ax[1].clear()
        points = self.ax[1].scatter(latitude, longitude, altitude, picker=True)
        self.ax[1].set_xlabel('Latitude')
        self.ax[1].set_ylabel('Longitude')
        self.ax[1].set_zlabel('Altitude (km)')
        self.ax[1].scatter([latitude[0]], [longitude[0]], [altitude[0]], marker='x', s=100, color='red')
        self.ax[1].text(latitude[0], longitude[0], altitude[0], "Start", color='red', fontsize=8, ha='right', va='bottom')
        self.ax[1].scatter([latitude[-1]], [longitude[-1]], [altitude[-1]], marker='x', s=100, color='red')
        self.ax[1].text(latitude[-1], longitude[-1], altitude[-1], "End", color='red', fontsize=8, ha='right', va='bottom')
        self.ax[1].xaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        self.ax[1].yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        self.ax[1].zaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        self.ax[1].text(0.02, 0.98, 0.98, 'Latitude ($\\times 10^6$ degrees)\nLongitude ($\\times 10^6$ degrees)\nAltitude ($\\times 10^3$ km)',
                        transform=self.ax[1].transAxes, fontsize=8, verticalalignment='top', horizontalalignment='left', bbox=dict(facecolor='white', alpha=0.8))
        plt.show()

    def on_nmea_data(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        if not np.isnan(latitude) and not np.isnan(longitude) and not np.isnan(altitude):
            rospy.loginfo(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")
            self.publish_gps_data(latitude, longitude, altitude)
        else:
            rospy.logwarn("Invalid NMEA data: Latitude, Longitude, or Altitude is not available.")

    def publish_gps_data(self, latitude, longitude, altitude):
        if not self.publisher:
            self.publisher = rospy.Publisher('/gps_plotter', NavSatFix, queue_size=10)

        msg = NavSatFix()
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude

        self.publisher.publish(msg)

    def parse_gga_sentence(self, sentence):
        pattern = r'\$GPGGA,\d+\.\d+,\d+\.\d+,[NS],\d+\.\d+,[EW],\d,\d+,\d+\.\d+,\d+\.\d+,M,\d+\.\d+,M,(\d+)'
        match = re.match(pattern, sentence)

        if match:
            latitude = float(match.group(2))
            latitude_direction = match.group(3)
            longitude = float(match.group(4))
            longitude_direction = match.group(5)
            altitude = float(match.group(9))
            timestamp = int(float(match.group(1)))
            seoul_timezone = timezone(timedelta(hours=9))
            utc_time = datetime.fromtimestamp(timestamp, tz=timezone.utc)
            seoul_time = utc_time.astimezone(seoul_timezone)

            print(f"UTC Seoul Time: {seoul_time}")
            print(f"Latitude: {latitude} {latitude_direction}")
            print(f"Longitude: {longitude} {longitude_direction}")
            print(f"Altitude: {altitude}")

            return [seoul_time], [latitude], [longitude], [altitude]

        else:
            print("Invalid GPGGA sentence format")
            return [], [], [], []

    def main(self):
        rospy.init_node('gps_plotter_node')

        rospy.Subscriber('/fix', NavSatFix, self.on_nmea_data)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('gps_plotter_node')
    gps_plotter = GPSPlotter()
    gps_plotter.main()
