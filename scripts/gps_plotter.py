#! /usr/bin/env python

import numpy as np
import math
import re
import datetime
import rospy
import serial
import logging
import math
import calendar
import time

from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d
from matplotlib.ticker import ScalarFormatter
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float32

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference

from nmea_msgs.msg import Sentence
from libnmea_navsat_driver.driver import RosNMEADriver

class GPSparser:
    logger = logging.getLogger('rosout')
    field_delimiter_regex = re.compile(r'[,*]')
    
    def __init__(self):
        self.Publisher = None
        self.fig, self.ax = plt.subplot(2, 1, figsize=(10,10))
        self.ser = serial.Serial()
    
    def safe_int(self, field):
        try:
            return int(field)
        except ValueError:
            return 0
    
    def safe_float(self, field):
        try:
            return float(field)
        except ValueError:
            return float('NaN')
        
    def convert_latitude(self, field):
        return self.safe_float(field[0:2]) + self.safe_float(field[2:]) / 60.0
    
    def convert_longitude(self, field):
        return self.safe_float(field[0:3]) + self.safe_float(field[3:]) / 60.0
    
    def convert_time(self, nmea_utc):
        if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
            return (float('NaN'), float('NaN'))
        
        utc_time = datetime.datetime.utcnow()
        hours = int(nmea_utc[0:2])
        minuites = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        nanosecs = 0
        
        if len(nmea_utc) > 7:
            nanosecs = int(nmea_utc[7:]) * pow(10, 9 - len(nmea_utc[7:]))
        
        day_offset = int((utc_time.hour - hours)/12.0)
        utc_time += datetime.timedelta(day_offset)
        utc_time = utc_time.replace(hour=hours, minute=minuites, second=seconds)
        unix_secs = calendar.timegm(utc_time.timetuple())
        return (unix_secs, nanosecs)
    
    def convert_time_rmc(date_str, time_str):
        if not date_str[0:6] or not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
            return (float('NaN'), float('NaN'))
    
        pc_year = datetime.date.today().year
        utc_year = int(date_str[4:6])
        years = pc_year + int((pc_year % 100 - utc_year) / 50.0)
        
        months = int(date_str[2:4])
        days = int(date_str[0:2])
        
        hours = int(time_str[0:2])
        minuites = int(time_str[2:4])
        seconds = int(time_str[4:6])
        nanosecs = int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))
        
        unix_secs = calendar.timegm((years, months, days, hours, minutes, secodns))
        return (unix_secs, nanosecs)
    
    def convert_knots_to_mps(self, knots):
        return self.safe_float(knots) * 0.514444444444
    
    def convert_status(status):
        if status == "A":
            return True
        if status == "V":
            return False
        else:
            return False
        
    
    def convert_deg_to_rad(self, degree):
        return math.radians(self.safe_float(degree))
    
    parsing_maps = {
        "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("fix_valid", convert_status, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rad, 8),
    ]
    }
    
    def parsing_nmea_sentence(self, nmea_sentence):
        nmea_sentence = nmea_sentence.strip()
        pattern = r'(^\$GP|^\$GN|^\$GL|^\$IN).*\*[0-9A-Fa-f]{2}$'
        if not re.match(pattern, nmea_sentence):
            self.logger.debug("Regex not match, is the sentence valid?\nSentence was : %s" % repr(nmea_sentence))
            return False
        fields = [field for field in self.field_delimiter_regex.split(nmea_sentence)]
        
        # Ignore the $GN $GP
        sentence_type = fields[0][3:]
        
        if sentence_type not in self.parsing_maps:
            self.logger.debug("Sentence type %s not in parsing maps" % repr(sentence_type))
            return False
        
        self.parsing_maps = self.parsing_maps[sentence_type]
        
        parsed_sentence = {}
        for entry in self.parsing_maps:
            parsed_sentence[entry[0]] = entry[1](fields[entry[2]])
        
        if sentence_type == "RMC":
            parsed_sentence["utc_time"] = self.convert_time_rmc(fields[9], fields[1])
        
        return {sentence_type: parsed_sentence}

    def check_nmea_check(nmea_sentence):
        split_sentence = nmea_sentence.split('*')
        if len(split_sentence) != 2:
            return False
        transmitted_check = split_sentence[1].strip()
        
        data_to_check = split_sentence[0][1:]
        check = 0
        for i in data_to_check:
            check ^= ord(i)
        
        return ("%02X" % check) == transmitted_check.upper()
    
    
            
            
class ROSdriver(object):
    def __init__(self):
        # self.fix_data = rospy.Publisher('gps', NavSatFix, queue_size=1)
        self.fix_data = rospy.Publisher('/gps', NavSatFix, queue_size=1)
        self.latitude_data = rospy.Publisher('/gps/latitude', Float32, queue_size=1)
        self.longitude_data = rospy.Publisher('/gps/longitude', Float32, queue_size=1)
        self.altitude_data = rospy.Publisher('/gps/altitude', Float32, queue_size=1)
        self.vel_data = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.heading_data = rospy.Publisher('heading', QuaternionStamped, queue_size=1)
        self.using_GNSS = rospy.get_param('~use_GNSS_time', False)
        if not self.using_GNSS:
            self.time_ref_data = rospy.Publisher('time_reference', TimeReference, queue_size=1)
        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.using_RMC = rospy.get_param('~useRMC', False)
        self.valid_fix = False
        
        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")
    
    
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not GPSparser.check_nmea_check(nmea_string):
            rospy.logwarn("Invalid Checksum.\n" + "Sentnece was : %s" % repr(nmea_string))
            return False
        parsed_sentence = GPSparser.parsing_nmea_sentence(nmea_string)
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
        if not self.using_GNSS:
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id
        
        if not self.using_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            data = parsed_sentence['GGA']
            
            if self.using_GNSS:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in NMEA sentence is not valid")
                    return False
            current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            
            fix_type = data['fix_type']
            self.valid_fix = (fix_type > 0)
            current_fix.status.status = NavSatStatus.SERVICE_GPS
            
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
        
            self.fix_data.publish(current_fix)
            
            self.latitude_data.publish(data['latitude'])
            self.longitude_data.publish(data['longitude'])
            self.altitude_data.publish(data['altitude'] + data['mean_sea_level'])

            if not (math.isnan(data['utc_time'][0]) or self.using_GNSS):
                current_time_ref.time_ref = rospy.Time(data['utc_time'][0], data['utc_time'][1])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_data.publish(current_time_ref)
        
        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']
            
            if self.using_GNSS:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in NMEA sentence is not valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            if self.using_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

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
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_data.publish(current_fix)

                if not (math.isnan(data['utc_time'][0]) or self.using_GNSS):
                    current_time_ref.time_ref = rospy.Time(
                        data['utc_time'][0], data['utc_time'][1])
                    self.time_ref_data.publish(current_time_ref)
                    
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_data.publish(current_vel)
        else:
            return False
    
    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else :
            return frame_id
    
    
    
    def nmea_sentence_callback(nmea_sentence, gps_parser, ros_driver):
        # NMEA 문장을 파싱하기 위해 GPSparser 클래스의 parsing_nmea_sentence 메서드 호출
        parsed_sentence = gps_parser.parsing_nmea_sentence(nmea_sentence)

        if parsed_sentence:
            # 만약 문장이 성공적으로 파싱되었다면, ROSdriver 클래스의 add_sentence 메서드 호출하여 메시지 발행
            ros_driver.add_sentence(nmea_sentence, frame_id='gps_frame')

    def main():
        rospy.init_node('gps_plotter')

        # GPSparser와 ROSdriver 클래스 인스턴스 생성
        gps_parser = GPSparser()
        ros_driver = ROSdriver()

        serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
        serial_baud = rospy.get_param('~baud', 9600)

        try:
            GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
            
            try:
                while not rospy.is_shutdown():
                    # 시리얼 포트로부터 데이터를 읽어옴
                    data = GPS.readline().strip()
                    try:
                        nmea_str = data.decode('ascii')
                        # NMEA 문장 콜백 함수 호출
                        ros_driver.nmea_sentence_callback(nmea_str, gps_parser, ros_driver)
                    except UnicodeError as e:
                        rospy.logwarn("ASCII 문자열로 디코딩할 수 없어 시리얼 장치에서 한 줄을 읽어오는 데 건너뜁니다. (바이트: {0})".format(data))
                    except ValueError as e:
                        rospy.logwarn("값 오류, NMEA 메시지에서 필드가 누락되었을 수 있습니다. 오류 내용: {0}".format(e))
            except (rospy.ROSInterruptException, serial.serialutil.SerialException):
                GPS.close()
        except serial.SerialException as ex:
            rospy.logfatal("시리얼 포트를 열 수 없습니다: I/O 오류({0}): {1}".format(ex.errno, ex.strerror))

