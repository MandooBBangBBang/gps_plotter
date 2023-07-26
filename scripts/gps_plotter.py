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
import re # regex
from datetime import datetime, timezone, timedelta


def format_altitude(altitude):
    return altitude / 1000

def format_coordinate(coord):
    return coord / 1e7


def plot_2D_graph(latitude, longitude):
    # 2D 그래프를 그리는 함수 구현
    # 데이터 포맷 변경
    latitude = [format_coordinate(lat) for lat in latitude]
    longitude = [format_coordinate(lon) for lon in longitude]

    # 2D 그래프 객체 생성
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # 2D 그래프에 데이터를 플롯
    ax.plot(latitude, longitude, marker='o', linestyle='-')

    # 축 라벨 설정 (단위 표시)
    ax.set_xlabel('Latitude')
    ax.set_ylabel('Longitude')

    # 시작점과 끝점을 그래프에 빨간 X 표시
    ax.scatter([latitude[0]], [longitude[0]], marker='x', s=100, color='red', label='Start')
    ax.scatter([latitude[-1]], [longitude[-1]], marker='x', s=100, color='red', label='End')

    # 그래프 출력
    plt.legend()
    plt.show()
    pass



def plot_3D_graph(latitude, longitude, altitude):
    # 3D 그래프 객체 생성
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 데이터 포맷 변경
    latitude = [format_coordinate(lat) for lat in latitude]
    longitude = [format_coordinate(lon) for lon in longitude]
    altitude = [format_altitude(alt) for alt in altitude]

    # 3D 그래프에 데이터를 플롯
    points = ax.scatter(latitude, longitude, altitude, picker=True)

    # 축 라벨 설정 (단위 표시)
    ax.set_xlabel('Latitude')
    ax.set_ylabel('Longitude')
    ax.set_zlabel('Altitude (km)')

    # 시작점과 끝점에 큰 빨간색 'X' 표시 (작은 글씨로 "Start", "End" 라벨 표시)
    ax.scatter([latitude[0]], [longitude[0]], [altitude[0]], marker='x', s=100, color='red')
    ax.text(latitude[0], longitude[0], altitude[0], "Start", color='red', fontsize=8, ha='right', va='bottom')

    ax.scatter([latitude[-1]], [longitude[-1]], [altitude[-1]], marker='x', s=100, color='red')
    ax.text(latitude[-1], longitude[-1], altitude[-1], "End", color='red', fontsize=8, ha='right', va='bottom')

    # 축 값의 지수 표기 제거하고 x10^6 등의 스케일링 표기 사용
    ax.xaxis.set_major_formatter(ScalarFormatter(useMathText=True))
    ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
    ax.zaxis.set_major_formatter(ScalarFormatter(useMathText=True))

    # 그래프 상단 왼쪽에 단위 표시 (폰트 크기 조정)
    ax.text(0.02, 0.98, 0.98, 'Latitude ($\\times 10^6$ degrees)\nLongitude ($\\times 10^6$ degrees)\nAltitude ($\\times 10^3$ km)',
            transform=ax.transAxes, fontsize=8, verticalalignment='top', horizontalalignment='left', bbox=dict(facecolor='white', alpha=0.8))

    def on_click(event):
        # 클릭한 점들의 index 가져오기
        ind = event.ind
        if len(ind) == 2:
            # 두 점만 선택했을 때 해당 점들 색 변경 (빨간색)
            points.set_color('blue')
            points._facecolors[ind] = (1, 0, 0, 1)

            # 점들의 헤딩 방향 표시 (두 점 사이만)
            for i in range(ind[0], ind[1]):
                ax.quiver(latitude[i], longitude[i], altitude[i],
                          latitude[i + 1] - latitude[i], longitude[i + 1] - longitude[i], altitude[i + 1] - altitude[i],
                          color='blue', length=0.1, arrow_length_ratio=0.2)

            # 보간하여 부드러운 선 그리기 (두 점 사이만)
            num_points = 100  # 선의 간격 조절을 위한 포인트 수
            interp_fn_lat = interp1d(np.arange(len(latitude))[ind[0]:ind[1]+1], latitude[ind[0]:ind[1]+1], kind='cubic')
            interp_fn_lon = interp1d(np.arange(len(longitude))[ind[0]:ind[1]+1], longitude[ind[0]:ind[1]+1], kind='cubic')
            interp_fn_alt = interp1d(np.arange(len(altitude))[ind[0]:ind[1]+1], altitude[ind[0]:ind[1]+1], kind='cubic')
            t_new = np.linspace(ind[0], ind[1], num_points)
            lat_new = interp_fn_lat(t_new)
            lon_new = interp_fn_lon(t_new)
            alt_new = interp_fn_alt(t_new)
            ax.plot(lat_new, lon_new, alt_new, color='red')

        else:
            # 두 점 이상 선택했을 때 초기화
            points.set_color('blue')
            ax.clear()
            plot_3D_graph(latitude, longitude, altitude)  # 변수들을 인자로 다시 전달하여 초기화

        # 그래프 갱신
        fig.canvas.draw_idle()

    # 클릭 이벤트를 위한 연결
    fig.canvas.mpl_connect('pick_event', on_click)

    # 전체 데이터에 대해 보간하여 부드러운 선 그리기
    num_points = 100  # 선의 간격 조절을 위한 포인트 수
    interp_fn_lat = interp1d(np.arange(len(latitude)), latitude, kind='cubic')
    interp_fn_lon = interp1d(np.arange(len(longitude)), longitude, kind='cubic')
    interp_fn_alt = interp1d(np.arange(len(altitude)), altitude, kind='cubic')
    t_new = np.linspace(0, len(latitude) - 1, num_points)
    lat_new = interp_fn_lat(t_new)
    lon_new = interp_fn_lon(t_new)
    alt_new = interp_fn_alt(t_new)
    ax.plot(lat_new, lon_new, alt_new, color='red')

    # 그래프 출력
    plt.show()
    pass

def on_nmea_data1(msg):
    
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude
    
    if not np.isnan(latitude) and not np.isnan(longitude) and not np.isnan(altitude):
        plot_2D_graph([latitude], [longitude])
        # 3D 그래프 그리기
        plot_3D_graph([latitude], [longitude], [altitude])
    else:
        print("Invalid NMEA data: Latitude, Longitude, or Altitude is not available.")



    
def parse_gga_sentence(sentence):
    # NMEA 문장을 파싱하여 위도, 경도, 고도 값을 추출하는 함수를 구현합니다.
    # 이 부분은 주어진 예시의 parse_gga_sentence 함수를 활용하겠습니다.
    pattern = r'\$GPGGA,\d+\.\d+,\d+\.\d+,[NS],\d+\.\d+,[EW],\d,\d+,\d+\.\d+,\d+\.\d+,M,\d+\.\d+,M,(\d+)'
    match = re.match(pattern, sentence)

    if match:
        latitude = float(match.group(2))
        latitude_direction = match.group(3)
        longitude = float(match.group(4))
        longitude_direction = match.group(5)
        altitude = float(match.group(9))
        timestamp = int(float(match.group(1)))
        
        # UTC 서울 시간으로 변환
        seoul_timezone = timezone(timedelta(hours=9))  # UTC +09:00
        utc_time = datetime.fromtimestamp(timestamp, tz=timezone.utc)
        seoul_time = utc_time.astimezone(seoul_timezone)

        print(f"Latitude: {latitude} {latitude_direction}")
        print(f"Longitude: {longitude} {longitude_direction}")
        print(f"Altitude: {altitude}")
        print(f"UTC Seoul Time: {seoul_time}")

        return [latitude], [longitude], [altitude], [seoul_time]

    else:
        print("Invalid GPGGA sentence format")
        return [], [], [], []


def main():
    
    rospy.init_node('gps_plotter_node')
    rospy.Subscriber('/fix', NavSatFix, on_nmea_data1)
    rospy.spin()

    


if __name__ == "__main__":
    main()