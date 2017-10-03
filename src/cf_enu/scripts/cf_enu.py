#!/usr/bin/python
# -*- coding: utf-8 -*-
### change coordinate frame GEO to ENU####
"""
Created on Tue Sep 13 12:02:53 2016

@author: Oscar Toku
"""
from __future__ import print_function
import sys
import rospy
import roslib
import tf
roslib.load_manifest('cf_enu')
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

def conversion(ini,final):
	ne,nn,nu=[1,2,3]

	return ne,nn,nu

class cf_enu:

	def __init__(self):
		self.gps_coords_sub=rospy.Subscriber("/navsat/fix",Odometry,self.conversion_callback,queue_size=1)
		self.imu_vals_sub=rospy.Subscriber("/imu/data ",Imu,self.orientation_callback,queue_size=1)

		self.gps_odometry_pub = rospy.Publisher('gps_odometry', String, queue_size=1)
		self.frame_id='GPS_ENU'
		self.child_frame_id='base_link'
		self.msg=Odometry()

	def orientation_callback(self,imu_val):
		q = imu_val.orientation
		self.msg.pose.pose.orientation=Quaternion(*q)
		if (self.orientation_received==False):
			self.orientation_received=True

	def conversion_callback(self, gps_coords):
		self.msg.header.stamp=rospy.Time.now()
		self.msg.header.frame_id=self.frame_id
		self.msg.child_fram_id=self.child_frame_id
		self.msg.pose.pose.position=Point(e,n,u)
		self.gps_odometry_pub.publish(self.msg)
		




def main(args):
	ic = cf_enu()
	rospy.init_node('cf_enu',anonymous=True)
	try:
	     rospy.spin()
	except KeyboardInterrupt:
	      pass

if __name__ == '__main__':
	main(sys.argv)
