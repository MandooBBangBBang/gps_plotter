#!/usr/bin/env python

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import ScalarFormatter
import numpy as np
from scipy.interpolate import interp1d
import re
from datetime import datetime, timezone, timedelta
import rospy
import serial
from std_msgs.msg import String


class GPSPlotter:
    def __init__(self):
        self.publisher = None
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 10))
        self.ser = serial.Serial()

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

    def get_utc_seoul_time(self):
        # 시스템 시간을 UTC로 얻어옴
        utc_time = datetime.utcnow()

        # UTC 시간을 서울 시간으로 변환
        seoul_timezone = timezone(timedelta(hours=9))  # UTC +09:00
        seoul_time = utc_time.astimezone(seoul_timezone)

        return seoul_time

    def publish_gps_data(self, utc_time, latitude, longitude, altitude):
        if not self.publisher:
            self.publisher = rospy.Publisher('gps_data', String, queue_size=10)

        # 데이터를 원하는 형식으로 가공하여 문자열로 만듦
        data_str = f"\nUTC Time : {utc_time}\nLatitude : {latitude:.6f} degrees\nLongitude : {longitude:.6f} degrees\nAltitude : {altitude:.3f} km\n---"

        self.publisher.publish(data_str)        

    def process_raw_data(self, raw_data):
        lines = raw_data.split('\n')
        for line in lines:
            line = line.strip()  # \r 제거
            if line.startswith('$GNGGA'):
                self.parse_gga_sentence(line)


    def on_nmea_data(self, msg):
        sentence = msg.data
        self.parse_gga_sentence(sentence)

    def parse_gga_sentence(self, sentence):
        # pattern = r'\$GNGGA,(\d+\.\d+),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),\d+,\d+,\d+\.\d+,\d+\.\d+,M'
        pattern = r'\$GNGGA,(\d+\.\d+),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),\d+,\d+,\d+,\d+,(\d+),M'
        match = re.match(pattern, sentence)

        if match:
            utc_time = float(match.group(1))
            latitude = float(match.group(2))
            latitude_direction = match.group(3)
            longitude = float(match.group(4))
            longitude_direction = match.group(5)
            altitude = float(match.group(9))

            # UTC 시간을 시, 분, 초로 변환
            utc_hour = int(utc_time / 10000)
            utc_minute = int((utc_time % 10000) / 100)
            utc_second = int(utc_time % 100)

            # 도, 분, 초로 변환된 값 출력
            rospy.loginfo(f"UTC Time: {utc_hour:02d}:{utc_minute:02d}:{utc_second:02d}")
            rospy.loginfo(f"Latitude: {latitude:.6f} {latitude_direction}")
            rospy.loginfo(f"Longitude: {longitude:.6f} {longitude_direction}")
            rospy.loginfo(f"Altitude: {altitude:.3f} meters")

            # GPS 데이터 발행
            self.publish_gps_data(f"{utc_hour:02d}:{utc_minute:02d}:{utc_second:02d}", latitude, longitude, altitude)

        else:
            rospy.logwarn("Invalid GNGGA sentence format")

    def receive_serial_data(self, data):
        data = data.data
        rospy.loginfo(f"Writing to serial port: {data}")
        self.ser.write(data.encode())


    def main(self):
        rospy.init_node('gps_plotter_node')
        rospy.Subscriber("gps_write", String, self.receive_serial_data)
        read_pub = rospy.Publisher("gps_data", String, queue_size=1000)

        try:
            ###########################
            # 여기 포트를 수정하세요! #
            ###########################
            self.ser.port = "/dev/ttyUSB0"
            self.ser.baudrate = 9600
            self.ser.timeout = 1
            self.ser.open()
        except serial.SerialException:
            rospy.logerr("Unable to open port. plz do chmod")
            return

        if self.ser.is_open:
            rospy.loginfo("Serial Port initialized")
        else:
            rospy.logerr("Failed to open port")
            return

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            data = self.ser.read_all().decode()
            if data:
                rospy.loginfo(f"Reading from serial port: {data}")
                read_pub.publish(data)
            rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node('gps_plotter_node')
    gps_plotter = GPSPlotter()
    rospy.Subscriber("gps_read", String, gps_plotter.publish_gps_data)
    rospy.Subscriber("gps_data", String, gps_plotter.on_nmea_data)
    gps_plotter.main()