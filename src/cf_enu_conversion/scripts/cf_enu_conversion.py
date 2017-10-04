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
import numpy as np
import roslib
import tf
roslib.load_manifest('cf_enu_conversion')
import time
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import Imu, NavSatFix

def conversion(llh):
	llh0=np.array([-99.12718,  19.47771,   0.     ])
	a = 6378137
	b = 6356752.3142
	e2 = 1 - (b/a)**2
	#%%%%%%%%%%Location of reference point in radians
	phi = llh0[1]*np.pi/180;
	lam = llh0[0]*np.pi/180;
	h = llh0[2]

	#%%%%%%%%%%Location of data points in radians
	dphi= llh[1]*np.pi/180 - phi;
	dlam= llh[0]*np.pi/180 - lam;
	dh = llh[2] - h;
	#%%%%%%%%%%Some useful definitions
	tmp1 = np.sqrt(1-e2*np.sin(phi)**2);
	cl = np.cos(lam);
	sl = np.sin(lam);
	cp = np.cos(phi);
	sp = np.sin(phi);
	#%%%%%%%%%%Transformations
	de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1**3)+h)*sp*dphi*dlam +cp*dlam*dh
	dn = (a*(1-e2)/tmp1**3 + h)*dphi + 1.5*cp*sp*a*e2*dphi**2 + sp**2*dh*dphi + 0.5*sp*cp*(a/tmp1 +h)*dlam**2
	du = dh - 0.5*(a-1.5*a*e2*cp**2+0.5*a*e2+h)*dphi**2 - 0.5*cp**2*(a/tmp1 -h)*dlam**2

	denu = np.array([de, dn, du])

	return denu

class cf_enu:
	def __init__(self):
		self.orientation_received=False
		self.gps_received=False
		print('paso')
		self.imu_vals_sub=rospy.Subscriber("/imu/data",Imu,self.orientation_callback,queue_size=1)
		self.gps_coords_sub=rospy.Subscriber("navsat/fix",NavSatFix,self.conversion_callback,queue_size=1)
		self.gps_odometry_pub = rospy.Publisher('gps_odometry', Odometry, queue_size=1)
		self.yaw_sub = rospy.Subscriber( "model_car/yaw2", Float32,self.headingcallback,queue_size=1)
		self.frame_id='GPS_ENU'
		self.child_frame_id='base_link'
		self.msg=Odometry()

	def headingcallback(self,dat):
      	self.yaw=dat.data
		
	def orientation_callback(self,imu_val):
		new_yaw=self.yaw
		q = imu_val.orientation
		dif=q.z-new_yaw
		if dif>180:
        	dif=dif-360
   		elif dif<-180:	
        	dif=dif+360
        self.msg.twist.twist.angular=Vector3(0,0,dif)
		self.msg.pose.pose.orientation=q
		if (self.orientation_received==False):
			self.orientation_received=True
		if self.orientation_received and self.gps_received:
			self.gps_odometry_pub.publish(self.msg)
			self.orientation_received=False
			self.gps_received=False
			
	def conversion_callback(self,gps_coords):
		self.msg.header.stamp=rospy.Time.now()
		self.msg.header.frame_id=self.frame_id
		self.msg.child_frame_id=self.child_frame_id
		latitud=gps_coords.latitude
		longitud=gps_coords.longitude
		altitud=gps_coords.altitude
		coords=np.array([longitud,latitud,altitud])
		enu=conversion(coords)
		self.msg.pose.pose.position=Point(enu[0],enu[1],enu[2])
		if self.gps_received==False:
			self.gps_received=True
		if self.orientation_received and self.gps_received:
			self.gps_odometry_pub.publish(self.msg)
			self.orientation_received=False
			self.gps_received=False



def main(args):
	rospy.init_node('cf_enu',anonymous=True)
	ic=cf_enu()
	
	try:
	     rospy.spin()
	except KeyboardInterrupt:
	      print("Shutting down")
	      pass

if __name__ == '__main__':
	main(sys.argv)
