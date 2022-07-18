#!/usr/bin/env python3
#-*- coding:utf-8 -*-	# 한글 주석을 달기 위해 사용한다.
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, PointCloud2
import socket
import time
from multiprocessing import Value, Process, Manager
import struct 
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
import math
import copy
import numpy as np
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension, Float32MultiArray
import serial
import threading

class Shared:
    def __init__(self):
        self.auto_manual = 0
        self.estop = 0
        self.gear = 0
        self.speed = 0
        self.steer = 0
        self.encoder = 0
        self.brake = 1
        self.alive = 0


class SerialRead(threading.Thread):
    def __init__(self, shared, ser, rate,  Gear_PCU,
                        SPEED_PCU,
                        STEER_PCU,
                        Brake_PCU,
                        Gear_Upper,
                        SPEED_Upper,
                        STEER_Upper,
                        Brake_Upper):
        super().__init__()
        self.shared = shared
        self.ser = ser
        self.interval = 1.0 / rate

    def run(self):

        P_data = {}

        ERP_cnt = 0
        while True:

            raw_serial = self.ser.read_until(b'\x0D\x0A')
            # raw_serial = ser.read(size=18)
            print(raw_serial)
            print(len(raw_serial))

            P_data['Header'] = raw_serial[0:3].decode('utf-8')
            P_data['AorM'] = raw_serial[3]
            P_data['E_stop'] = raw_serial[4]
            P_data['Gear'] = raw_serial[5]
            P_data['Speed'] = struct.unpack('<H', raw_serial[6:8])[0]
            P_data['Steer'] = struct.unpack('<h', raw_serial[8:10])[0]
            P_data['Brake'] = raw_serial[10]
            # P_data['Encoder_0'] = raw_serial[11]
            # P_data['Encoder_1'] = raw_serial[12]
            # P_data['Encoder_2'] = raw_serial[13]
            # P_data['Encoder_3'] = raw_serial[14]
            P_data['Alv_Cnt'] = raw_serial[11]

            print(P_data)

            State = [P_data['AorM'], P_data['E_stop'], P_data['Gear'], P_data['Speed'], P_data['Steer'],
                     P_data['Brake'], P_data['Alv_Cnt']]

            # Encoder = [P_data['Encoder_0'], P_data['Encoder_1'], P_data['Encoder_2'], P_data['Encoder_3'],
            #            P_data['Alv_Cnt']]

            time.sleep(self.interval)

        # while True:
        #
        #     packet = self.ser.read_until(b'\x0d\x0a')
        #     print(len(packet))
        #     # print(self.ser.read())
        #
        #     header = packet[0:3].decode()
        #
        #     # if header == "STX":
        #     #     # auto_manual, e-stop, gear
        #     #     (self.shared.automanual, self.shared.estop, self.shared.gear) = struct.unpack("BBB", packet[3:6])
        #     #
        #     #     # speed, steer
        #     #     tmp1, tmp2 = struct.unpack("2h", packet[6:10])
        #     #     self.shared.speed = tmp1 / 10  # km/h
        #     #     self.shared.steer = tmp2 / 71  # degree
        #     #
        #     #     # brake
        #     #     self.shared.brake = struct.unpack("B", packet[10:11])[0]
        #     #
        #     #     # encoder
        #     #     self.shared.encoder = struct.unpack("f", packet[11:15])
        #     #
        #     #     # alive (heartbeat)
        #     #     self.shared.alive = struct.unpack("B", packet[15:16])[0]
        #     #
        #     #     Gear_PCU.value = self.shared.gear
        #     #     SPEED_PCU.value = self.shared.speed
        #     #     STEER_PCU.value = self.shared.steer
        #     #     Brake_PCU.value = self.shared.brake
        #
        #
        #     time.sleep(self.interval)


class SerialWrite(threading.Thread):
    def __init__(self, shared, ser, rate,
                 Gear_PCU,
                 SPEED_PCU,
                 STEER_PCU,
                 Brake_PCU,
                 Gear_Upper,
                 SPEED_Upper,
                 STEER_Upper,
                 Brake_Upper
                 ):
        super().__init__()
        self.shared = shared
        self.ser = ser
        self.interval = 1.0 / rate

    def run(self):
        while True:


            self.shared.gear = Gear_Upper.value
            self.shared.speed = SPEED_Upper.value
            self.shared.steer = STEER_Upper.value
            self.shared.brake = Brake_Upper.value
            self.shared.brake  = 1
            self.shared.steer = 0.09
            # self.shared.speed = 5
            result = bytearray()
            result = b'\x53' + b'\x54' + b'\x58'

            result = result + struct.pack('B', 1)

            result = result + struct.pack('B', self.shared.estop)

            result = result + struct.pack('B', int(self.shared.gear))

            result = result + struct.pack('>H', int(self.shared.speed * 10))

            result = result + struct.pack('>h', int(self.shared.steer * 71))

            result = result + struct.pack('B', int(self.shared.brake))

            result = result + struct.pack('B', self.shared.alive)

            result = result + b'\x0D' + b'\x0A'

            # print(result)

            # cnt must put!!!!
            self.shared.alive = self.shared.alive + 1

            if self.shared.alive == 256:
                self.shared.alive = 0

            self.ser.write(result)

            time.sleep(self.interval)

def chatterCallback(data):

    current_accel_x.value, current_accel_y.value, current_accel_z.value = round(data.linear_acceleration.x,2), round(data.linear_acceleration.y,2), round(data.linear_acceleration.z,2)
    
    current_vel_x.value, current_vel_y.value,current_vel_z.value = round(data.angular_velocity.x,2), round(data.angular_velocity.y,2), round(data.angular_velocity.z,2)

    current_quat_x.value, current_quat_y.value, current_quat_z.value, current_quat_w.value = round(data.orientation.x,2), round(data.orientation.y,2), round(data.orientation.z,2), round(data.orientation.w,2)

    quat_list = [current_quat_x.value, current_quat_y.value,current_quat_z.value, current_quat_w.value]

    roll, pitch, yaw = euler_from_quaternion(quat_list)
    
    IMU_CTC.value = 1
    
    # 이유는 모르겠으나 def GPSINS에서 yaw값을 산출하면 속도는 엄청나게 빠르나 중간에 계속 렉 걸리면서 컴퓨터가 멈춰버림
    # 안정성을 위해서 callback에서 처리하는게 나을까?
    error_yaw = 17
    
    yaw = math.degrees(yaw) + error_yaw

    current_yaw.value = round(yaw,2)

def FusionCallback(data):
    
    data = data.data
    final_fusion_obj = []

    a = [0 for k in range(7)]

    if len(data) == 4:
        final_fusion_obj = [a for i in range(7)]
        pass
    else:

        for i in range(int((len(data) - 4) / 28)): ##dd

            f_obj = data[28 * i + 4 : 28 * i + 32]
            f_obj_list = []

            for i in range(7):
                
                num = round(float(f_obj[4 * i : 4 * i + 4]) * 0.01,2)

                if float(f_obj[4 * i + 3])%2==1:
                    if i == 6:
                        num *= 100  
                        num = int(num) #1~8
                    else:
                        num = num*(-1) 
                               
                else:
                    pass
                #print(num) #include minus, display number
                f_obj_list.append(num)


            final_fusion_obj.append(f_obj_list)
        
        num = 7 - len(final_fusion_obj)

        for i in range(num):
            final_fusion_obj.append(a)
    
    if (len(data) == 32) : 
        f_obj1_x_cent.value = final_fusion_obj[0][0]
        f_obj1_y_cent.value = final_fusion_obj[0][1]
        f_obj1_x_min.value  = final_fusion_obj[0][2]
        f_obj1_x_max.value  = final_fusion_obj[0][3]
        f_obj1_y_min.value  = final_fusion_obj[0][4]
        f_obj1_y_max.value  = final_fusion_obj[0][5]
        f_obj1_class.value  = final_fusion_obj[0][6]
        f_obj2_x_cent.value = 0
        f_obj2_y_cent.value = 0
        f_obj2_x_min.value  = 0
        f_obj2_x_max.value  = 0
        f_obj2_y_min.value  = 0
        f_obj2_y_max.value  = 0
        f_obj2_class.value  = 0
        f_obj3_x_cent.value = 0
        f_obj3_y_cent.value = 0
        f_obj3_x_min.value  = 0
        f_obj3_x_max.value  = 0
        f_obj3_y_min.value  = 0
        f_obj3_y_max.value  = 0
        f_obj3_class.value  = 0
        f_obj4_x_cent.value = 0
        f_obj4_y_cent.value = 0
        f_obj4_x_min.value  = 0
        f_obj4_x_max.value  = 0
        f_obj4_y_min.value  = 0
        f_obj4_y_max.value  = 0
        f_obj4_class.value  = 0
        #print("1111111111111111")
        # print("x center : ", final_fusion_obj[0][0])
        # print("y center : ", final_fusion_obj[0][1])
        # print("x min : ", final_fusion_obj[0][2])
        # print("x max : ", final_fusion_obj[0][3])
        # print("y min : ", final_fusion_obj[0][4])
        # print("y max : ", final_fusion_obj[0][5])
        # print("class : ", final_fusion_obj[0][6])
    
    elif (len(data) == 60) : 
        f_obj1_x_cent.value = final_fusion_obj[0][0]
        f_obj1_y_cent.value = final_fusion_obj[0][1]
        f_obj1_x_min.value  = final_fusion_obj[0][2]
        f_obj1_x_max.value  = final_fusion_obj[0][3]
        f_obj1_y_min.value  = final_fusion_obj[0][4]
        f_obj1_y_max.value  = final_fusion_obj[0][5]
        f_obj1_class.value  = final_fusion_obj[0][6]
        f_obj2_x_cent.value = final_fusion_obj[1][0]
        f_obj2_y_cent.value = final_fusion_obj[1][1]
        f_obj2_x_min.value  = final_fusion_obj[1][2]
        f_obj2_x_max.value  = final_fusion_obj[1][3]
        f_obj2_y_min.value  = final_fusion_obj[1][4]
        f_obj2_y_max.value  = final_fusion_obj[1][5]
        f_obj2_class.value  = final_fusion_obj[1][6]
        f_obj3_x_cent.value = 0
        f_obj3_y_cent.value = 0
        f_obj3_x_min.value  = 0
        f_obj3_x_max.value  = 0
        f_obj3_y_min.value  = 0
        f_obj3_y_max.value  = 0
        f_obj3_class.value  = 0
        f_obj4_x_cent.value = 0
        f_obj4_y_cent.value = 0
        f_obj4_x_min.value  = 0
        f_obj4_x_max.value  = 0
        f_obj4_y_min.value  = 0
        f_obj4_y_max.value  = 0
        f_obj4_class.value  = 0
        #print("222222222222222")
    elif (len(data) == 88) : 
        f_obj1_x_cent.value = final_fusion_obj[0][0]
        f_obj1_y_cent.value = final_fusion_obj[0][1]
        f_obj1_x_min.value  = final_fusion_obj[0][2]
        f_obj1_x_max.value  = final_fusion_obj[0][3]
        f_obj1_y_min.value  = final_fusion_obj[0][4]
        f_obj1_y_max.value  = final_fusion_obj[0][5]
        f_obj1_class.value  = final_fusion_obj[0][6]
        f_obj2_x_cent.value = final_fusion_obj[1][0]
        f_obj2_y_cent.value = final_fusion_obj[1][1]
        f_obj2_x_min.value  = final_fusion_obj[1][2]
        f_obj2_x_max.value  = final_fusion_obj[1][3]
        f_obj2_y_min.value  = final_fusion_obj[1][4]
        f_obj2_y_max.value  = final_fusion_obj[1][5]
        f_obj2_class.value  = final_fusion_obj[1][6]
        f_obj3_x_cent.value = final_fusion_obj[2][0]
        f_obj3_y_cent.value = final_fusion_obj[2][1]
        f_obj3_x_min.value  = final_fusion_obj[2][2]
        f_obj3_x_max.value  = final_fusion_obj[2][3]
        f_obj3_y_min.value  = final_fusion_obj[2][4]
        f_obj3_y_max.value  = final_fusion_obj[2][5]
        f_obj3_class.value  = final_fusion_obj[2][6]
        #print("3333333333333333333")
    elif (len(data) == 116) : 
        f_obj1_x_cent.value = final_fusion_obj[0][0]
        f_obj1_y_cent.value = final_fusion_obj[0][1]
        f_obj1_x_min.value  = final_fusion_obj[0][2]
        f_obj1_x_max.value  = final_fusion_obj[0][3]
        f_obj1_y_min.value  = final_fusion_obj[0][4]
        f_obj1_y_max.value  = final_fusion_obj[0][5]
        f_obj1_class.value  = final_fusion_obj[0][6]
        f_obj2_x_cent.value = final_fusion_obj[1][0]
        f_obj2_y_cent.value = final_fusion_obj[1][1]
        f_obj2_x_min.value  = final_fusion_obj[1][2]
        f_obj2_x_max.value  = final_fusion_obj[1][3]
        f_obj2_y_min.value  = final_fusion_obj[1][4]
        f_obj2_y_max.value  = final_fusion_obj[1][5]
        f_obj2_class.value  = final_fusion_obj[1][6]
        f_obj3_x_cent.value = final_fusion_obj[2][0]
        f_obj3_y_cent.value = final_fusion_obj[2][1]
        f_obj3_x_min.value  = final_fusion_obj[2][2]
        f_obj3_x_max.value  = final_fusion_obj[2][3]
        f_obj3_y_min.value  = final_fusion_obj[2][4]
        f_obj3_y_max.value  = final_fusion_obj[2][5]
        f_obj3_class.value  = final_fusion_obj[2][6]
        f_obj4_x_cent.value = final_fusion_obj[3][0]
        f_obj4_y_cent.value = final_fusion_obj[3][1]
        f_obj4_x_min.value  = final_fusion_obj[3][2]
        f_obj4_x_max.value  = final_fusion_obj[3][3]
        f_obj4_y_min.value  = final_fusion_obj[3][4]
        f_obj4_y_max.value  = final_fusion_obj[3][5]
        f_obj4_class.value  = final_fusion_obj[3][6]
        #print("4444444444444444")
    

    # Traffic Light CTC 
    for i in range(4):
        if final_fusion_obj[i][6] == 9:
            light_r = 1
        else:
            light_r = 0

        if final_fusion_obj[i][6] == 10:
            light_o = 1
        else:
            light_o = 0

        if final_fusion_obj[i][6] == 11:
            light_g = 1
        else:
            light_g = 0

        if final_fusion_obj[i][6] == 12:
            light_lr = 1
        else:
            light_lr = 0

        if final_fusion_obj[i][6] == 13:
            light_lo = 1
        else:
            light_lo = 0

        if final_fusion_obj[i][6] == 14:
            light_lg = 1
        else:
            light_lg = 0    

    # Fusion Object CTC
    if (len(data) == 32):
        f_obj1_CTC.value = 1
    else  : f_obj1_CTC.value = 0   

    if (len(data) == 60):
        f_obj2_CTC.value = 1
    else  : f_obj2_CTC.value = 0   

    if (len(data) == 88):
        f_obj3_CTC.value = 1
    else  : f_obj3_CTC.value = 0   

    if len(data) == 116:
        f_obj4_CTC.value = 1
    else  : f_obj4_CTC.value = 0       


    FUSION_CTC.value = 1

def LidarCallback(data):

   
    LIDAR_CTC.value = 1
    final_obj = []

    data = data.data

    a = [0 for k in range(7)]


    k = int((len ( data ) - 4) / 28 ) # no touch!!!!

    if len(data) == 4:

        final_obj = [a for i in range(7)]
        pass
    else:
        
        for i in range(int((len ( data ) - 4) / 28 )):

            obj = data[28 * i + 4 : 28 * i + 32]
            obj_list = []

        for i in range(k):

            obj = data[28 * i + 4 : 28 * i + 32]
            obj_list = []

            for i in range(7):
               
                num = round(float(obj[4 * i : 4 * i + 4]) * 0.01,2)

                if float(obj[4 * i + 3])%2==1:
                    num = num*(-1)                    
                else:
                    pass
                #print(num) #include minus, display number
                obj_list.append(num)

            final_obj.append(obj_list)
  
        num = 7 - len(final_obj)

        for i in range(num):
            final_obj.append(a)

        
    obj1_dist.value   = final_obj[0][0]
    obj1_x_cent.value = final_obj[0][1]
    obj1_y_cent.value = final_obj[0][2]
    obj1_x_min.value  = final_obj[0][3]
    obj1_x_max.value  = final_obj[0][4]
    obj1_y_min.value  = final_obj[0][5]
    obj1_y_max.value  = final_obj[0][6]
    
    #print("obj1 dist :", obj1_dist.value)
    obj2_dist.value   = final_obj[1][0]
    obj2_x_cent.value = final_obj[1][1]
    obj2_y_cent.value = final_obj[1][2]
    obj2_x_min.value  = final_obj[1][3]
    obj2_x_max.value  = final_obj[1][4]
    obj2_y_min.value  = final_obj[1][5]
    obj2_y_max.value  = final_obj[1][6]

    obj3_dist.value   = final_obj[2][0]
    obj3_x_cent.value = final_obj[2][1]
    obj3_y_cent.value = final_obj[2][2]
    obj3_x_min.value  = final_obj[2][3]
    obj3_x_max.value  = final_obj[2][4]
    obj3_y_min.value  = final_obj[2][5]
    obj3_y_max.value  = final_obj[2][6]

    obj4_dist.value   = final_obj[3][0]
    obj4_x_cent.value = final_obj[3][1]
    obj4_y_cent.value = final_obj[3][2]
    obj4_x_min.value  = final_obj[3][3]
    obj4_x_max.value  = final_obj[3][4]
    obj4_y_min.value  = final_obj[3][5]
    obj4_y_max.value  = final_obj[3][6]

    LIDAR_CTC.value = 1
    

def GNSS_Callback(data):

    

    kalman_lat.value, kalman_lon.value, kalman_alt.value = data.data[0],data.data[1],data.data[2]
    current_lat.value, current_lon.value, current_alt.value = data.data[3],data.data[4],data.data[5]
    kalman_roll.value, kalman_pitch.value, kalman_yaw.value = data.data[6],data.data[7],data.data[8]
    current_accel_x.value, current_accel_y.value, current_accel_z.value = data.data[9],data.data[10],data.data[11]
    current_vel_x.value, current_vel_y.value, current_vel_z.value = data.data[12],data.data[13],data.data[14]
    current_quat_x.value, current_quat_y.value, current_quat_z.value = data.data[15],data.data[16],data.data[17]
    current_quat_w.value = data.data[18]
    GPS_NED_N.value, GPS_NED_E.value, GPS_NED_D.value = data.data[19],data.data[20],data.data[21]
    GPS_ENU_E.value, GPS_ENU_N.value, GPS_ENU_U.value= data.data[22],data.data[23],data.data[24]
    print(kalman_yaw.value)
    
def GPSCallback(data):

    current_lat.value, current_lon.value, current_alt.value = data.latitude, data.longitude, data.altitude     
    GPS_CTC.value = 1
    
def GPSINS(current_lat, current_lon, current_alt, current_yaw,
    obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    obj1_x_cent,obj1_y_cent,obj1_x_min,obj1_x_max,obj1_y_min,obj1_y_max,
    obj2_x_cent,obj2_y_cent,obj2_x_min,obj2_x_max,obj2_y_min,obj2_y_max,
    obj3_x_cent,obj3_y_cent,obj3_x_min,obj3_x_max,obj3_y_min,obj3_y_max,
    obj4_x_cent,obj4_y_cent,obj4_x_min,obj4_x_max,obj4_y_min,obj4_y_max,
    IMU_CTC, GPS_CTC, LIDAR_CTC, FUSION_CTC,
    f_obj1_CTC,f_obj2_CTC,f_obj3_CTC,f_obj4_CTC,
    f_obj1_class, f_obj2_class, f_obj3_class, f_obj4_class,
    light_r, light_o, light_g, light_lr, light_lo, light_lg,kalman_yaw,
           shared
           ):

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("vectornav/IMU", Imu, chatterCallback)

    rospy.Subscriber("Lidar_msg", String, LidarCallback)

    rospy.Subscriber("Fusion_msg", String, FusionCallback) 

    rospy.Subscriber("ublox/fix", NavSatFix, GPSCallback)

    rospy.Subscriber("/INS", Float32MultiArray, GNSS_Callback)

    rospy.spin()


def tcpip(current_lat, current_lon, current_alt, current_yaw,
    obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    f_obj1_x_cent,f_obj1_y_cent,f_obj1_x_min,f_obj1_x_max,f_obj1_y_min,f_obj1_y_max,
    f_obj2_x_cent,f_obj2_y_cent,f_obj2_x_min,f_obj2_x_max,f_obj2_y_min,f_obj2_y_max,
    f_obj3_x_cent,f_obj3_y_cent,f_obj3_x_min,f_obj3_x_max,f_obj3_y_min,f_obj3_y_max,
    f_obj4_x_cent,f_obj4_y_cent,f_obj4_x_min,f_obj4_x_max,f_obj4_y_min,f_obj4_y_max,
    IMU_CTC, GPS_CTC, LIDAR_CTC, FUSION_CTC,
    f_obj1_CTC,f_obj2_CTC,f_obj3_CTC,f_obj4_CTC,
    f_obj1_class, f_obj2_class, f_obj3_class, f_obj4_class,
    light_r, light_o, light_g, light_lr, light_lo, light_lg ,kalman_yaw,shared, Gear_PCU,
                         SPEED_PCU,
                         STEER_PCU,
                         Brake_PCU,
                         Gear_Upper,
                         SPEED_Upper,
                         STEER_Upper,
                         Brake_Upper):
    #host = "192.168.1.99" # 서버 컴퓨터의 ip(여기선 내 컴퓨터를 서버 컴퓨터로 사용) 
                       # 본인의 ip주소 넣어도 됨(확인방법: cmd -> ipconfig)
    port = 4567  # 서버 포트번호(다른 프로그램이 사용하지 않는 포트번호로 지정해야 함)


    # host = "192.168.1.77"
    host = "192.168.10.22"

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port)) # try communication

    server_socket.listen(1)
    print('echo server start')

    # 클라이언트 접속 기다리며 대기
    client_soc, addr = server_socket.accept()

    msg = struct.pack('>54d',
                      current_lat.value, current_lon.value, current_alt.value, kalman_yaw.value,
                      obj1_dist.value, obj2_dist.value, obj3_dist.value, obj4_dist.value,
                      f_obj1_x_cent.value, f_obj1_y_cent.value, f_obj1_x_min.value, f_obj1_x_max.value,
                      f_obj1_y_min.value, f_obj1_y_max.value,
                      f_obj2_x_cent.value, f_obj2_y_cent.value, f_obj2_x_min.value, f_obj2_x_max.value,
                      f_obj2_y_min.value, f_obj2_y_max.value,
                      f_obj3_x_cent.value, f_obj3_y_cent.value, f_obj3_x_min.value, f_obj3_x_max.value,
                      f_obj3_y_min.value, f_obj3_y_max.value,
                      f_obj4_x_cent.value, f_obj4_y_cent.value, f_obj4_x_min.value, f_obj4_x_max.value,
                      f_obj4_y_min.value, f_obj4_y_max.value,
                      IMU_CTC.value, GPS_CTC.value, LIDAR_CTC.value, FUSION_CTC.value,
                      f_obj1_CTC.value, f_obj2_CTC.value, f_obj3_CTC.value, f_obj4_CTC.value,
                      f_obj1_class.value, f_obj2_class.value, f_obj3_class.value, f_obj4_class.value,
                      light_r.value, light_o.value, light_g.value, light_lr.value, light_lo.value, light_lg.value,
                      Gear_PCU.value,
                      SPEED_PCU.value,
                      STEER_PCU.value,
                      Brake_PCU.value
                      )

    client_soc.sendall(msg)

    print('connected client addr:', addr)
    print("통신 진행 중")
    cnt = 0
    # 클라이언트가 보낸 패킷 계속 받아 에코메세지 돌려줌
    while True:
        time.sleep(0.01)

        cnt = cnt + 1
        #
        if current_lat.value == 0 or current_lon.value == 0 or current_alt.value == 0:
            print("GPS No Signal")
            data = client_soc.recv(1024)

            data = struct.unpack(">4d",data)

            # Gear_Upper.value  = data[0]
            # SPEED_Upper.value = data[1]
            # STEER_Upper.value = data[2]
            # Brake_Upper.value = data[3]

            msg = struct.pack('>54d',
                current_lat.value, current_lon.value, current_alt.value, kalman_yaw.value,
                obj1_dist.value,obj2_dist.value,obj3_dist.value,obj4_dist.value,
                f_obj1_x_cent.value,f_obj1_y_cent.value,f_obj1_x_min.value,f_obj1_x_max.value,f_obj1_y_min.value,f_obj1_y_max.value,
                f_obj2_x_cent.value,f_obj2_y_cent.value,f_obj2_x_min.value,f_obj2_x_max.value,f_obj2_y_min.value,f_obj2_y_max.value,
                f_obj3_x_cent.value,f_obj3_y_cent.value,f_obj3_x_min.value,f_obj3_x_max.value,f_obj3_y_min.value,f_obj3_y_max.value,
                f_obj4_x_cent.value,f_obj4_y_cent.value,f_obj4_x_min.value,f_obj4_x_max.value,f_obj4_y_min.value,f_obj4_y_max.value,
                IMU_CTC.value, GPS_CTC.value, LIDAR_CTC.value, FUSION_CTC.value,
                f_obj1_CTC.value,f_obj2_CTC.value,f_obj3_CTC.value,f_obj4_CTC.value,
                f_obj1_class.value, f_obj2_class.value, f_obj3_class.value, f_obj4_class.value,
                light_r.value, light_o.value, light_g.value, light_lr.value, light_lo.value, light_lg.value,
                              Gear_PCU.value,
                              SPEED_PCU.value,
                              STEER_PCU.value,
                              Brake_PCU.value
            )

            client_soc.sendall(msg)

            pass
        else:

            data = client_soc.recv()

            data = struct.unpack("d", data)
            Gear_Upper.value = data[0]
            SPEED_Upper.value = data[1]
            STEER_Upper.value = data[2]
            Brake_Upper.value = data[3]

            msg = struct.pack('>54d',
                              current_lat.value, current_lon.value, current_alt.value, kalman_yaw.value,
                              obj1_dist.value, obj2_dist.value, obj3_dist.value, obj4_dist.value,
                              f_obj1_x_cent.value, f_obj1_y_cent.value, f_obj1_x_min.value, f_obj1_x_max.value,
                              f_obj1_y_min.value, f_obj1_y_max.value,
                              f_obj2_x_cent.value, f_obj2_y_cent.value, f_obj2_x_min.value, f_obj2_x_max.value,
                              f_obj2_y_min.value, f_obj2_y_max.value,
                              f_obj3_x_cent.value, f_obj3_y_cent.value, f_obj3_x_min.value, f_obj3_x_max.value,
                              f_obj3_y_min.value, f_obj3_y_max.value,
                              f_obj4_x_cent.value, f_obj4_y_cent.value, f_obj4_x_min.value, f_obj4_x_max.value,
                              f_obj4_y_min.value, f_obj4_y_max.value,
                              IMU_CTC.value, GPS_CTC.value, LIDAR_CTC.value, FUSION_CTC.value,
                              f_obj1_CTC.value, f_obj2_CTC.value, f_obj3_CTC.value, f_obj4_CTC.value,
                              f_obj1_class.value, f_obj2_class.value, f_obj3_class.value, f_obj4_class.value,
                              light_r.value, light_o.value, light_g.value, light_lr.value, light_lo.value,
                              light_lg.value,
                              Gear_PCU.value,
                              SPEED_PCU.value,
                              STEER_PCU.value,
                              Brake_PCU.value
                              )
            client_soc.sendall(msg)

    print("통신 끝")
    server_socket.close() # 사용했던 서버 소켓을 닫아줌"""

if __name__ == '__main__':
    current_lat = Value('d', 0.0)
    current_lon = Value('d', 0.0)
    current_alt = Value('d', 0.0)
    current_accel_x = Value('d', 0.0)
    current_accel_y = Value('d', 0.0)
    current_accel_z = Value('d', 0.0)
    current_vel_x = Value('d', 0.0)
    current_vel_y = Value('d', 0.0)
    current_vel_z = Value('d', 0.0)
    current_quat_x = Value('d', 0.0)
    current_quat_y = Value('d', 0.0)
    current_quat_z = Value('d', 0.0)
    current_quat_w = Value('d', 0.0)
    current_yaw = Value('d', 0.0)

    current_roll = Value('d', 0.0)
    current_pitch = Value('d', 0.0)

    kalman_yaw = Value('d', 0.0)
    kalman_pitch = Value('d', 0.0)
    kalman_roll = Value('d', 0.0)

    kalman_lat = Value('d', 0.0)
    kalman_lon = Value('d', 0.0)
    kalman_alt = Value('d', 0.0)

    kalman_NED_N = Value('d', 0.0)
    kalman_NED_E = Value('d', 0.0)
    kalman_NED_D = Value('d', 0.0)

    kalman_ENU_E = Value('d', 0.0)
    kalman_ENU_N = Value('d', 0.0)
    kalman_ENU_U = Value('d', 0.0)

    GPS_NED_N = Value('d', 0.0)
    GPS_NED_E = Value('d', 0.0)
    GPS_NED_D = Value('d', 0.0)

    GPS_ENU_E = Value('d', 0.0)
    GPS_ENU_N = Value('d', 0.0)
    GPS_ENU_U = Value('d', 0.0)

    #obj = [dist,x_cent, y_cent, x_min,x_max, y_min, y_max]
    obj1_dist = Value('d', 0.0)
    obj2_dist = Value('d', 0.0)
    obj3_dist = Value('d', 0.0)
    obj4_dist = Value('d', 0.0)

    obj1_x_cent = Value('d', 0.0)
    obj2_x_cent = Value('d', 0.0)
    obj3_x_cent = Value('d', 0.0)
    obj4_x_cent = Value('d', 0.0)

    obj1_y_cent = Value('d', 0.0)
    obj2_y_cent = Value('d', 0.0)
    obj3_y_cent = Value('d', 0.0)
    obj4_y_cent = Value('d', 0.0)

    obj1_x_min = Value('d', 0.0)
    obj2_x_min = Value('d', 0.0)
    obj3_x_min = Value('d', 0.0)
    obj4_x_min = Value('d', 0.0)

    obj1_x_max = Value('d', 0.0)
    obj2_x_max = Value('d', 0.0)
    obj3_x_max = Value('d', 0.0)
    obj4_x_max = Value('d', 0.0)

    obj1_y_min = Value('d', 0.0)
    obj2_y_min = Value('d', 0.0)
    obj3_y_min = Value('d', 0.0)
    obj4_y_min = Value('d', 0.0)

    obj1_y_max = Value('d', 0.0)
    obj2_y_max = Value('d', 0.0)
    obj3_y_max = Value('d', 0.0)
    obj4_y_max = Value('d', 0.0)

    IMU_CTC    = Value('d', 0.0)
    GPS_CTC    = Value('d', 0.0)
    LIDAR_CTC  = Value('d', 0.0)
    FUSION_CTC  = Value('d', 0.0)
    
    LIDAR_obj_1 = Value('d', 0.0)
    LIDAR_obj_2 = Value('d', 0.0)
    LIDAR_obj_3 = Value('d', 0.0)
    LIDAR_obj_4 = Value('d', 0.0)
    
    f_obj1_x_cent = Value('d', 0.0)
    f_obj2_x_cent = Value('d', 0.0)
    f_obj3_x_cent = Value('d', 0.0)
    f_obj4_x_cent = Value('d', 0.0)

    f_obj1_y_cent = Value('d', 0.0)
    f_obj2_y_cent = Value('d', 0.0)
    f_obj3_y_cent = Value('d', 0.0)
    f_obj4_y_cent = Value('d', 0.0)

    f_obj1_x_min = Value('d', 0.0)
    f_obj2_x_min = Value('d', 0.0)
    f_obj3_x_min = Value('d', 0.0)
    f_obj4_x_min = Value('d', 0.0)

    f_obj1_x_max = Value('d', 0.0)
    f_obj2_x_max = Value('d', 0.0)
    f_obj3_x_max = Value('d', 0.0)
    f_obj4_x_max = Value('d', 0.0)

    f_obj1_y_min = Value('d', 0.0)
    f_obj2_y_min = Value('d', 0.0)
    f_obj3_y_min = Value('d', 0.0)
    f_obj4_y_min = Value('d', 0.0)

    f_obj1_y_max = Value('d', 0.0)
    f_obj2_y_max = Value('d', 0.0)
    f_obj3_y_max = Value('d', 0.0)
    f_obj4_y_max = Value('d', 0.0)

    f_obj1_CTC = Value('d', 0.0)
    f_obj2_CTC = Value('d', 0.0)
    f_obj3_CTC = Value('d', 0.0)
    f_obj4_CTC = Value('d', 0.0)

    f_obj1_class = Value('d', 0.0)
    f_obj2_class = Value('d', 0.0)
    f_obj3_class = Value('d', 0.0)
    f_obj4_class = Value('d', 0.0)

    light_r = Value('d', 0.0)
    light_o = Value('d', 0.0)
    light_g = Value('d', 0.0)
    light_lr = Value('d', 0.0)
    light_lo = Value('d', 0.0)
    light_lg = Value('d', 0.0)

    Gear_PCU    = Value('d', 0.0)
    SPEED_PCU   = Value('d', 0.0)
    STEER_PCU   = Value('d', 0.0)
    Brake_PCU   = Value('d', 0.0)

    Gear_Upper  = Value('d', 0.0)
    SPEED_Upper = Value('d', 0.0)
    STEER_Upper = Value('d', 0.0)
    Brake_Upper = Value('d', 0.0)



    ser = serial.Serial("/dev/ttyUSB0", 115200)
    shared = Shared()
    reader = SerialRead(shared, ser, 20,
                        Gear_PCU,
                        SPEED_PCU,
                        STEER_PCU,
                        Brake_PCU,
                        Gear_Upper,
                        SPEED_Upper,
                        STEER_Upper,
                        Brake_Upper

    )
    writer = SerialWrite(shared, ser, 20,
                         Gear_PCU,
                         SPEED_PCU,
                         STEER_PCU,
                         Brake_PCU,
                         Gear_Upper,
                         SPEED_Upper,
                         STEER_Upper,
                         Brake_Upper
                         )
    reader.daemon = True
    writer.daemon = True

    # th1 = Process(target=GPSINS, args=(current_lat, current_lon, current_alt, current_yaw,
    # obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    # f_obj1_x_cent,f_obj1_y_cent,f_obj1_x_min,f_obj1_x_max,f_obj1_y_min,f_obj1_y_max,
    # f_obj2_x_cent,f_obj2_y_cent,f_obj2_x_min,f_obj2_x_max,f_obj2_y_min,f_obj2_y_max,
    # f_obj3_x_cent,f_obj3_y_cent,f_obj3_x_min,f_obj3_x_max,f_obj3_y_min,f_obj3_y_max,
    # f_obj4_x_cent,f_obj4_y_cent,f_obj4_x_min,f_obj4_x_max,f_obj4_y_min,f_obj4_y_max,
    # IMU_CTC, GPS_CTC, LIDAR_CTC, FUSION_CTC,
    # f_obj1_CTC,f_obj2_CTC,f_obj3_CTC,f_obj4_CTC,
    # f_obj1_class, f_obj2_class, f_obj3_class, f_obj4_class,
    # light_r, light_o, light_g, light_lr, light_lo, light_lg ,kalman_yaw,
    #                                    shared
    #                                    ))
    # #
    # th2 = Process(target=tcpip, args=(current_lat, current_lon, current_alt, current_yaw,
    # obj1_dist,obj2_dist,obj3_dist,obj4_dist,
    # f_obj1_x_cent,f_obj1_y_cent,f_obj1_x_min,f_obj1_x_max,f_obj1_y_min,f_obj1_y_max,
    # f_obj2_x_cent,f_obj2_y_cent,f_obj2_x_min,f_obj2_x_max,f_obj2_y_min,f_obj2_y_max,
    # f_obj3_x_cent,f_obj3_y_cent,f_obj3_x_min,f_obj3_x_max,f_obj3_y_min,f_obj3_y_max,
    # f_obj4_x_cent,f_obj4_y_cent,f_obj4_x_min,f_obj4_x_max,f_obj4_y_min,f_obj4_y_max,
    # IMU_CTC, GPS_CTC, LIDAR_CTC, FUSION_CTC,
    # f_obj1_CTC,f_obj2_CTC,f_obj3_CTC,f_obj4_CTC,
    # f_obj1_class, f_obj2_class, f_obj3_class, f_obj4_class,
    # light_r, light_o, light_g, light_lr, light_lo, light_lg ,kalman_yaw,
    #                                shared, Gear_PCU,
    #                      SPEED_PCU,
    #                      STEER_PCU,
    #                      Brake_PCU,
    #                      Gear_Upper,
    #                      SPEED_Upper,
    #                      STEER_Upper,
    #                      Brake_Upper
    #  ))
    #
    # th1.start()
    # th2.start()
    reader.start()
    writer.start()

    while True:

        time.sleep(0.05)

