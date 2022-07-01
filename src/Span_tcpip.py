#!/usr/bin/env python
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, PointCloud2
import socket
import time
from multiprocessing import Value, Process, Manager,Queue
import struct 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from novatel_gps_msgs.msg import NovatelPosition,NovatelUtmPosition,NovatelVelocity,NovatelXYZ,NovatelCorrectedImuData,NovatelDualAntennaHeading,NovatelHeading2,Inspva,Insstdev,Time

import numpy as np

def bestposCallback(data):

    if bestpos_th.value == 0:
        # to prevent overflow
        bestpos_q.empty()

        bestpos_q.put(data)

        bestpos_th.value = 1

    else: # bestpos_th == 1:
        pass

def bestxyzCallback(data):

    if bestxyz_th.value == 0:
        # to prevent overflow
        bestxyz_q.empty()

        bestxyz_q.put(data)

        bestxyz_th.value = 1
    else: # bestpos_th == 1:
        pass


def corrimudataCallback(data):

    if corrimudata_th.value == 0:
        # to prevent overflow
        corrimudata_q.empty()

        corrimudata_q.put(data)

        corrimudata_th.value = 1

    else:  # bestpos_th == 1:
        pass

def inspvaCallback(data):

    if inspva_th.value == 0:
        # to prevent overflow
        inspva_q.empty()

        inspva_q.put(data)

        inspva_th.value = 1
    else:  # bestpos_th == 1:
        pass


def GPSINS():

    rospy.init_node('span_tcpip', anonymous=True)

    rospy.Subscriber("bestpos", NovatelPosition, bestposCallback)

    rospy.Subscriber("bestxyz", NovatelXYZ, bestxyzCallback)

    rospy.Subscriber("/corrimudata", NovatelCorrectedImuData, corrimudataCallback)

    rospy.Subscriber("/inspva", Inspva, inspvaCallback)

    rospy.spin()


def tcpip(bestpos_q,bestpos_th,bestxyz_q,bestxyz_th,corrimudata_q,
                                      corrimudata_th,inspva_q,inspva_th):
    
    #host = "192.168.1.99" # 서버 컴퓨터의 ip(여기선 내 컴퓨터를 서버 컴퓨터로 사용)
                       # 본인의 ip주소 넣어도 됨(확인방법: cmd -> ipconfig)
    port = 4567  # 서버 포트번호(다른 프로그램이 사용하지 않는 포트번호로 지정해야 함)
    host = "192.168.1.70"
    #host = "127.0.0.1"
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # server_socket.bind((host, port))

    # 클라이언트 접속 준비 완료
    #server_socket.listen(1)

    print('echo server start')

    #  클라이언트 접속 기다리며 대기
    # client_soc, addr = server_socket.recvfrom(1)

    addr = (host, 4567)
    print('connected client addr:', addr)
    print("통신 진행 중")
    # 클라이언트가 보낸 패킷 계속 받아 에코메세지 돌려줌
    while True:

        if corrimudata_th.value == 1 and inspva_th.value == 1:
            corrimudata_th.value = 0
            inspva_th.value = 0

            corrimudata = corrimudata_q.get()
            inspva = inspva_q.get()

            if bestpos_th.value == 1 and bestxyz_th.value == 1:
                bestpos_th.value = 0
                bestxyz_th.value = 0

                bestpos = bestpos_q.get()
                bestxyz = bestxyz_q.get()

                break


    while True:

        if corrimudata_th.value == 1 and inspva_th.value == 1:
            corrimudata_th.value = 0
            inspva_th.value = 0

            corrimudata = corrimudata_q.get()
            inspva = inspva_q.get()

            if bestpos_th.value == 1 and bestxyz_th.value == 1:

                bestpos_th.value = 0
                bestxyz_th.value = 0

                bestpos = bestpos_q.get()
                bestxyz = bestxyz_q.get()

            msg = struct.pack('>31d',
                              bestpos.lat,
                              bestpos.lon,
                              bestpos.height,
                              bestpos.undulation,
                              bestpos.lat_sigma,
                              bestpos.lon_sigma,
                              bestpos.height_sigma,
                              bestpos.diff_age,
                              bestpos.solution_age,
                              bestxyz.x,
                              bestxyz.y,
                              bestxyz.z,
                              bestxyz.x_vel,
                              bestxyz.y_vel,
                              bestxyz.z_vel,
                              corrimudata.pitch_rate,
                              corrimudata.roll_rate,
                              corrimudata.yaw_rate,
                              corrimudata.lateral_acceleration,
                              corrimudata.longitudinal_acceleration,
                              corrimudata.vertical_acceleration,
                              inspva.seconds,
                              inspva.latitude,
                              inspva.longitude,
                              inspva.height,
                              inspva.north_velocity,
                              inspva.east_velocity,
                              inspva.up_velocity,
                              inspva.roll,
                              inspva.pitch,
                              inspva.azimuth)

            server_socket.sendto(msg, addr)
            #data = server_socket.recv(1)


            time.sleep(0.01)


        else:
            pass

            # msg = struct.pack('>31d', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            #                   0, 0, 0, 0)
            #
            # client_soc.sendall(msg)
            #
            # data = client_soc.recv(1)
            #
            # time.sleep(0.01)

    print("통신 끝")
    server_socket.close() # 사용했던 서버 소켓을 닫아줌"""


if __name__ == '__main__':
    bestpos_q = Queue()
    bestpos_th = Value('d',0)

    bestxyz_q = Queue()
    bestxyz_th = Value('d',0)

    corrimudata_q = Queue()
    corrimudata_th = Value('d',0)

    inspva_q = Queue()
    inspva_th = Value('d',0)


    th1 = Process(target=GPSINS, args=())

    th2 = Process(target=tcpip, args=(bestpos_q,bestpos_th,bestxyz_q,bestxyz_th,corrimudata_q,
                                      corrimudata_th,inspva_q,inspva_th
    ))

    th1.start()
    th2.start()



    
    
