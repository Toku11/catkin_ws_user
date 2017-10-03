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
roslib.load_manifest('gps')
import serial
import struct
from std_msgs.msg import String
from std_msgs.msg import Float32
ser1=serial.Serial('/dev/rfcomm0',115200)
ser1.flushInput()
ser1.flushOutput()


class gps:

	def __init__(self):
		self.pub = rospy.Publisher('gps', String, queue_size=1)



	def code(self):

		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			data_pix = ser1.read()
			if data_pix==">":
				data_pix=ser1.readline()
				data_list = data_pix.split(",")
				#a=len(data_list)
				#print(a)
				#print(data_list)
				tipo=float(data_list[0])
				x=float(data_list[2])
				y=float(data_list[3])
				z=float(data_list[4])
				if tipo==1:#ACCELEROMETER  (m/s^2 - X,Y,Z)
					acc=True
				elif tipo==2:#MAGNETIC_FIELD (uT - X,Y,Z)
					mag=True
				elif tipo==3:#ORIENTATION (Yaw, Pitch, Roll)
					orien=True
					quaternion=tf.transformations.quaternion_from_euler(x,y,z)
					print(quaternion[1])
				elif tipo==4:#GYROSCOPE (rad/sec - X,Y,Z)
					gyro=True
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
				elif tipo==99:#GPS2 (Bearing, Speed, Date/Time)
					gps2=True
				print('tipo=',tipo,'x=',x,'y=',y,'z=',z)


def main(args):
	ic = gps()
	rospy.init_node('gps',anonymous=True)
	try:
	     ic.code()
	except KeyboardInterrupt:
	      pass

if __name__ == '__main__':
	main(sys.argv)
