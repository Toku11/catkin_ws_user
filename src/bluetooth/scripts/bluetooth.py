#!/usr/bin/python
# -*- coding: utf-8 -*-
### Bluetooth con sensoduino####
"""
Created on Tue Sep 13 12:02:53 2016

@author: Oscar Toku
"""
from __future__ import print_function
import sys
import rospy
import roslib
import tf
roslib.load_manifest('bluetooth')
import serial
import struct
from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, MagneticField
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Quaternion, Vector3
ser1=serial.Serial('/dev/rfcomm0',115200)
ser1.flushInput()
ser1.flushOutput()


class bluetooth:

    def __init__(self):
        self.tf_broadcast = tf.TransformBroadcaster()
        self.pub_imu = rospy.Publisher('imu/data',Imu, queue_size=1)
        self.pub_navsatfix = rospy.Publisher('navsat/fix',NavSatFix,queue_size=1)
        self.pub_mag = rospy.Publisher('mag/data',MagneticField, queue_size=1)


    def code(self):
        micro=1000000
        acc_l=False
        gyro=False
        orien=False
        quat=Quaternion()
        imuMsg=Imu()
        gpsMsg=NavSatFix()
        magMsg=MagneticField()
        #rate = rospy.Rate(1000)
        last=rospy.Time.now().to_sec()
        imuMsg.orientation_covariance = [0 , 0 , 0,0, 0, 0,0, 0, 0]
        imuMsg.angular_velocity_covariance = [0, 0 , 0,0 , 0, 0,0, 0 , 0.02]
        imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,0 , 0.2, 0,0 , 0 , 0.2]
        gpsMsg.position_covariance = [0,0,0,0,0,0,0,0,0]
        while not rospy.is_shutdown():
            data_pix = ser1.read()
            if data_pix==">":
                data_pix=ser1.readline()
                data_list = data_pix.split(",")
                #a=len(data_list)
                #print(a)
                #print(data_list)
                tipo=float(data_list[0])
                xx=float(data_list[2])
                yy=float(data_list[3])
                zz=float(data_list[4])
                if tipo==1:#ACCELEROMETER  (m/s^2 - X,Y,Z)
                    acc=True
                elif tipo==2:#MAGNETIC_FIELD (uT - X,Y,Z)
                    mag=True
                    magMsg.header.stamp=rospy.Time.now()
                    magMsg.header.frame_id='base_link'
                    magMsg.magnetic_field.x= xx/micro
                    magMsg.magnetic_field.y= yy/micro
                    magMsg.magnetic_field.z= zz/micro
                    self.pub_mag.publish(magMsg)
                elif tipo==3:#ORIENTATION (Yaw, Pitch, Roll)
                    orien=True
                    # orientation to ENU Right Hand 
                    roll=-zz#*3.14159/180
                    pitch=yy#*3.18159/180
                    yaw=-xx+90#*3.14159/180
                    if yaw <-180:
                        yaw=yaw+360
                    #q=tf.transformations.quaternion_from_euler(roll*3.14159/180,pitch*3.18159/180,yaw*3.14159/180)
                    #imuMsg.orientation = Quaternion(*q)
                    imuMsg.orientation.x = roll #magnetometer
                    imuMsg.orientation.y = pitch
                    imuMsg.orientation.z = yaw
                    imuMsg.orientation.w = 0
                    #q=tf.transformations.quaternion_from_euler(zz*3.14159/180,yy*3.18159/180,xx*3.14159/180)'''
                    #quat.x = q[0] #magnetometer
                    #quat.y = q[1]
                    #quat.z = q[2]
                    #quat.w = q[3]
                elif tipo==4:   #GYROSCOPE (rad/sec - X,Y,Z)
                    gyro=True
                    imuMsg.angular_velocity.x = xx #gyro
                    imuMsg.angular_velocity.y = yy
                    imuMsg.angular_velocity.z = zz
                elif tipo==5:#LIGHT (SI lux)
                    lux=True
                elif tipo==6:#PRESSURE (hPa millibar)
                    press=True
                elif tipo==7:#DEVICE TEMPERATURE (C)
                    temp=True
                elif tipo==8:#PROXIMITY (Centimeters or 1,0)
                    prox=True
                elif tipo==9:#GRAVITY (m/s^2 - X,Y,Z)
                    grav=True
                elif tipo==10:#LINEAR_ACCELERATION (m/s^2 - X,Y,Z)
                    acc_l=True
                    imuMsg.linear_acceleration.x = xx # tripe axis accelerator meter
                    imuMsg.linear_acceleration.y = yy
                    imuMsg.linear_acceleration.z = zz
                elif tipo==11:#ROTATION_VECTOR (Degrees - X,Y,Z)
                    rot=True
                elif tipo==12:#RELATIVE_HUMIDITY (%)
                    hum=True
                elif tipo==13:#AMBIENT_TEMPERATURE (C)
                    amb_temp=True
                elif tipo==14:#MAGNETIC_FIELD_UNCALIBRATED (uT - X,Y,Z)
                    mag_u=True
                elif tipo==15:#GAME_ROTATION_VECTOR (Degrees - X,Y,Z)
                    game_rot=True
                elif tipo==16:#GYROSCOPE_UNCALIBRATED (rad/sec - X,Y,Z)
                    gyro_u=True
                elif tipo==17:#SIGNIFICANT_MOTION (1,0)
                    sig_mot=True
                elif tipo==97:#AUDIO (Vol.)}
                    audio=True
                elif tipo==98:#GPS1 (Lat., Long., Alt.)
                    gps1=True
                    gpsMsg.header.stamp=rospy.Time.now()
                    gpsMsg.header.frame_id='base_link'
                    gpsMsg.latitude = xx
                    gpsMsg.longitude = yy
                    gpsMsg.altitude = zz
                    gpsMsg.position_covariance_type = 0
                    gpsMsg.status.service=NavSatStatus.SERVICE_GPS
                    gpsMsg.status.status = NavSatStatus.STATUS_SBAS_FIX#SOLO SAT--STATUS_GBAS_FIX ANTENA TIERRA
                    self.pub_navsatfix.publish(gpsMsg)
                elif tipo==99:#GPS2 (Bearing, Speed, Date/Time)
                    gps2=True
                #print('tipo=',tipo,'x=',xx,'y=',yy,'z=',zz,rospy.Time.now().to_sec()-last)
                #rate.sleep()
                ## until the msg is complete
            if orien==True and acc_l==True and gyro==True:

                imuMsg.header.stamp=rospy.Time.now()
                imuMsg.header.frame_id='base_link'
                self.pub_imu.publish(imuMsg)
                acc_l=False
                gyro=False
                orien=False

def main(args):
    ic = bluetooth()
    rospy.init_node('bluetooth',anonymous=True)
    try:
         ic.code()
    except KeyboardInterrupt:
          pass

if __name__ == '__main__':
    main(sys.argv)
