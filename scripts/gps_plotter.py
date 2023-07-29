#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import re
import rospy
import serial
import logging
import math
import calendar
import datetime
import time

from scipy.interpolate import interp1d
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import ScalarFormatter

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler

# NMEA 형식의 데이터 파싱, 데이터 추출
class GPSparser:
    logger = logging.getLogger('rosout')
    
    def __init__(self):
        self.field_delimiter_regex = re.compile(r'[,*]')
    
    def format_altitude(self, altitude):
        return altitude / 1000

    def format_coordinate(self, coord): 
        return coord / 1e7

    def convert_latitude(latitude_str):
        degrees = float(latitude_str[:2])
        minutes = float(latitude_str[2:])
        return degrees + minutes / 60.0

    def convert_longitude(longitude_str):
        degrees = float(longitude_str[:3])
        minutes = float(longitude_str[3:])
        return degrees + minutes / 60.0
    
    def safe_float(value_str):
        try:
            return float(value_str)
        except ValueError:
            return None
        
    def safe_int(value_str):
        try:
            return int(value_str)
        except ValueError:
            return None

    @staticmethod
    def convert_time(time_str):
        hour = time_str[:2]
        minute = time_str[2:4]
        second = time_str[4:6]
        return f"{hour}:{minute}:{second}"
    
    @staticmethod   
    def convert_time_rmc(date_str, time_str):
        if not date_str[0:6] or not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
            return (float('NaN'), float('NaN'))

        pc_year = datetime.date.today().year

        utc_year = int(date_str[4:6])
        years = pc_year + int((pc_year % 100 - utc_year) / 50.0)

        months = int(date_str[2:4])
        days = int(date_str[0:2])

        hours = int(time_str[0:2])
        minutes = int(time_str[2:4])
        seconds = int(time_str[4:6])
        nanosecs = int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))

        unix_secs = calendar.timegm((years, months, days, hours, minutes, seconds))
        return (unix_secs, nanosecs)
    
    def parse_gga_sentence(self, fields):
        data = {
            'utc_time': self.convert_time(fields[1]),
            'latitude': self.convert_latitude(fields[2]),
            'latitude_direction': fields[3],
            'longitude': self.convert_longitude(fields[4]),
            'longitude_direction': fields[5],
            'fix_type': self.safe_int(fields[6]),
            'num_satellites': self.safe_int(fields[7]),
            'hdop': self.safe_float(fields[8]),
            'altitude': self.safe_float(fields[9]),
            'mean_sea_level': self.safe_float(fields[11]),
        }
        return data
    
    def parse_rmc_sentence(self, fields):
        data = {
            'utc_time': self.convert_time_rmc(fields[9], fields[1]),
            'latitude': self.convert_latitude(fields[3]),
            'latitude_direction': fields[4],
            'longitude': self.convert_longitude(fields[5]),
            'longitude_direction': fields[6],
            'fix_valid': self.convert_status_flag(fields[2]),
            'speed': self.convert_knots_to_mps(fields[7]),
            'true_course': self.convert_deg_to_rads(fields[8]),
        }
        return data

    # nmea_sentence 파싱후 데이터를 추출하고 딕셔너리 형태로 반환.
    def parse_nmea_sentence(self, nmea_sentence):
        nmea_sentence = nmea_sentence.strip()
        if not re.match(r'(^\$GPGGA|^\$GNGGA|^\$GNRMC).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
            self.logger.debug("Regex didn't match, sentence not valid NMEA? Sentence was: %s" % repr(nmea_sentence))
            return False

        fields = [field for field in self.field_delimiter_regex.split(nmea_sentence)]

        # $ 뒤의 두글자 무시 ex. $GN , $GP
        sentence_type = fields[0][3:]

        if sentence_type == 'GGA':
            return {sentence_type: self.parse_gga_sentence(fields)}
        elif sentence_type == 'RMC':
            return {sentence_type: self.parse_rmc_sentence(fields)}
        else:
            self.logger.debug("Sentence type %s not in parse map, ignoring." % repr(sentence_type))
            return False

    def check_nmea_checksum(self, nmea_sentence):
        split_sentence = nmea_sentence.split('*')
        if len(split_sentence) != 2:
            return False
        transmitted_checksum = split_sentence[1].strip()

        data_to_checksum = split_sentence[0][1:]
        checksum = 0
        for c in data_to_checksum:
            checksum ^= ord(c)

        return ("%02X" % checksum) == transmitted_checksum.upper()

# GPS 데이터를 ROS메시지로 변환 및 퍼블리싱
class ROSdriver:
    def __init__(self):
        self.fix_pub = rospy.Publisher('GPSdata', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('velocity', TwistStamped, queue_size=1)
        self.heading_pub = rospy.Publisher('heading', QuaternionStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        
        self.parser = GPSparser()

    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not GPSparser.check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = GPSparser.parse_nmea_sentence(nmea_string)

        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id

        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        sentence_type = list(parsed_sentence.keys())[0]

        if sentence_type == 'GGA':
            data = parsed_sentence['GGA']

            fix_type = data['fix_type']
            current_fix.status.status = NavSatStatus.STATUS_FIX
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2

            self.fix_pub.publish(current_fix)

            if not (math.isnan(data['utc_time'][0])):
                current_time_ref.time_ref = rospy.Time(data['utc_time'][0], data['utc_time'][1])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif sentence_type == 'RMC':
            data = parsed_sentence['RMC']

            current_fix.status.status = NavSatStatus.STATUS_FIX

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            current_fix.altitude = float('NaN')
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.fix_pub.publish(current_fix)

            if not (math.isnan(data['utc_time'][0])):
                current_time_ref.time_ref = rospy.Time(data['utc_time'][0], data['utc_time'][1])
                self.time_ref_pub.publish(current_time_ref)

        else:
            return False

    @staticmethod
    def get_frame():
        frame_id = rospy.get_param('~frame_id', 'gps')
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
        
    def receive_serial_data(self, msg):
        data = self.ser.read_all().decode()
        if data:
            rospy.loginfo(f"Reading from serial port: {data}")
            frame_id = "gps_frame"
            timestamp = rospy.Time.now()
            self.add_sentence(data, frame_id, timestamp)
        

    def on_nmea_data(self, msg):
        sentence = msg.data
        parsed_data = GPSparser.parse_nmea_sentence(sentence)
        
        if parsed_data:
            sentence_type = list(parsed_data.keys())[0]
            data = parsed_data[sentence_type]

            if sentence_type == 'GGA':
                # Extract data from the GGA sentence
                utc_time = data['utc_time']
                latitude = data['latitude']
                longitude = data['longitude']
                altitude = data['altitude']

                # Format the UTC time
                formatted_time = self.format_utc_time(utc_time)

                # Format the latitude, longitude, and altitude
                formatted_latitude = f"Latitude: {latitude:.6f}"
                formatted_longitude = f"Longitude: {longitude:.6f}"
                formatted_altitude = f"Altitude: {altitude:.2f}"

                # Print the formatted data
                print(formatted_time)
                print(formatted_latitude)
                print(formatted_longitude)
                print(formatted_altitude)
                print("---------------------------------------------")

            elif sentence_type == 'RMC':
                # Extract data from the RMC sentence
                utc_time = data['utc_time']
                latitude = data['latitude']
                longitude = data['longitude']

                # Format the UTC time
                formatted_time = self.format_utc_time(utc_time)

                # Format the latitude and longitude
                formatted_latitude = f"Latitude: {latitude:.6f}"
                formatted_longitude = f"Longitude: {longitude:.6f}"

                # Print the formatted data
                print(formatted_time)
                print(formatted_latitude)
                print(formatted_longitude)
                print("---------------------------------------------")
    
    @staticmethod
    def format_utc_time(utc_time):
        unix_secs, nanosecs = utc_time
        time_struct = time.gmtime(unix_secs)
        time_struct = time_struct._replace(tm_hour=time_struct.tm_hour + 9)
        formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time_struct)
        return f"UTC Time: {formatted_time}"


class GPSPlotter:    
    def __init__(self):
        rospy.init_node('gps_plotter')
        rospy.Subscriber("gps_write", NavSatFix, ROSdriver.receive_serial_data)
        
        try:
            ###########################
            # 여기 포트를 수정하세요! #
            ###########################
            self.ser = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
        except serial.SerialException:
            rospy.logerr("Unable to open port. Please make sure the port is accessible.")
            return

        if self.ser.is_open:
            rospy.loginfo("Serial Port initialized\n")
        else:
            rospy.logerr("Failed to open port")
            return

        self.rate = rospy.Rate(5)

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

# 모든 클래스를 인스턴스화
class MainNode:
    def __init__(self):
        self.gps_plotter = ROSdriver()
        rospy.Subscriber("gps_write", NavSatFix, self.gps_plotter.on_nmea_data)

if __name__ == "__main__":
    rospy.init_node('main_node')
    main_node = MainNode()
    rospy.spin()