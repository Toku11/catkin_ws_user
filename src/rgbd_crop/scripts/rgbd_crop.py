#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo 
import time
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import geometry_msgs.msg
class rgbd_crop:
    def __init__(self):
	#tfBuffer = tf2_ros.Buffer()
	#listener = tf2_ros.TransformListener(tfBuffer)
        br = tf2_ros.StaticTransformBroadcaster()
        t =geometry_msgs.msg.TransformStamped()
        t1 =geometry_msgs.msg.TransformStamped()
        stamp=rospy.Time.now()
        t.header.stamp = stamp
        t.header.frame_id = "camera_link"
        t.child_frame_id = "rgb_frame"
        t.transform.rotation.w = 1.0
        t1.header.stamp = stamp
        t1.header.frame_id = "rgb_frame"
        t1.child_frame_id = "rgb"
        t1.transform.rotation.x = -0.5
        t1.transform.rotation.y = 0.5
        t1.transform.rotation.z = -0.5
        t1.transform.rotation.w = 0.5
        d =geometry_msgs.msg.TransformStamped()
        d1 =geometry_msgs.msg.TransformStamped()
        d.header.stamp = stamp
        d.header.frame_id = "camera_link"
        d.child_frame_id = "depth_frame"
        d.transform.translation.x = 0.00380906066857
        d.transform.translation.y = -0.0246999990195
        d.transform.translation.z = 0.000733263499569
        d.transform.rotation.w = 1.0
        d1.header.stamp = stamp
        d1.header.frame_id = "depth_frame"
        d1.child_frame_id = "depth"
        d1.transform.rotation.x = -0.5
        d1.transform.rotation.y = 0.5
        d1.transform.rotation.z = -0.5
        d1.transform.rotation.w = 0.5
        br.sendTransform([t,t1,d,d1])
        self.bridge = CvBridge() 
        self.sub_cam_raw=rospy.Subscriber("camera_info_raw", CameraInfo, self.cam_info_raw_callback,queue_size=1)
        self.sub_cam_depth=rospy.Subscriber("camera_info_depth", CameraInfo, self.cam_info_depth_callback,queue_size=1)    

          #We are using a depth image to get depth information of what we're tracking.
        rospy.Subscriber("depth_image", Image, self.depth_callback,queue_size=1)
        rospy.Subscriber("rgb_image", Image, self.camera2_callback,queue_size=1)
        self.cam_pub = rospy.Publisher("crop/rgb/image_raw", Image,queue_size=1)
        self.depth_pub=rospy.Publisher("crop/depth/image_raw", Image,queue_size=1)
        self.cam_info_raw=rospy.Publisher("crop/rgb/camera_info", CameraInfo,queue_size=1)
        self.cam_info_depth=rospy.Publisher("crop/depth/camera_info", CameraInfo,queue_size=1)
	
    
    def cam_info_raw_callback(self, camera_info):
        self.cam_info = CameraInfo()
        self.cam_info.header.frame_id='rgb'
        self.cam_info.width = camera_info.width/2
        self.cam_info.height = camera_info.height/2
        self.cam_info.K = tuple(x/2 for x in camera_info.K)
        self.cam_info.D = camera_info.D
        self.cam_info.R = camera_info.R
        self.cam_info.P = camera_info.P
        self.cam_info.distortion_model = camera_info.distortion_model
        self.cam_info.binning_x = camera_info.binning_x
        self.cam_info.binning_y = camera_info.binning_y
        self.cam_info.roi.x_offset = camera_info.roi.x_offset
        self.cam_info.roi.y_offset = camera_info.roi.y_offset
        self.cam_info.roi.height = camera_info.roi.height
        self.cam_info.roi.width = camera_info.roi.width
        self.cam_info.roi.do_rectify = camera_info.roi.do_rectify
        self.sub_cam_raw.unregister()
        
    def cam_info_depth_callback(self, camera_info):

        self.cam_info1 = CameraInfo()
        self.cam_info1.header.frame_id='depth'
        self.cam_info1.width = camera_info.width/2
        self.cam_info1.height = camera_info.height/2
        self.cam_info1.K = tuple(x/2 for x in camera_info.K)
        self.cam_info1.D = camera_info.D
        self.cam_info1.R = camera_info.R
        self.cam_info1.P = camera_info.P
        self.cam_info1.distortion_model = camera_info.distortion_model
        self.cam_info1.binning_x = camera_info.binning_x
        self.cam_info1.binning_y = camera_info.binning_y
        self.cam_info1.roi.x_offset = camera_info.roi.x_offset
        self.cam_info1.roi.y_offset = camera_info.roi.y_offset
        self.cam_info1.roi.height = camera_info.roi.height
        self.cam_info1.roi.width = camera_info.roi.width
        self.cam_info1.roi.do_rectify = camera_info.roi.do_rectify
        self.sub_cam_depth.unregister()
        
    def camera2_callback(self, imagen):
	blur=20
	intensity=0.5
        imagen_cv1 = self.bridge.imgmsg_to_cv2(imagen, imagen.encoding)
        imagen_cv1 = cv2.resize(imagen_cv1, (640/2,480/2)) 
	blur=20
    	intensity=0.5/25
    	ligth=cv2.blur(imagen_cv1,(blur,blur)) 
    	ligth=cv2.cvtColor(ligth,cv2.COLOR_BGR2GRAY)
    	lab=cv2.cvtColor(imagen_cv1,cv2.COLOR_BGR2LAB)
    	l,a,b=cv2.split(lab)
    	l=l-ligth*intensity
    	imagen_cv=np.dstack((l,a,b))
    	imagen_cv=cv2.cvtColor(np.uint8(imagen_cv),cv2.COLOR_LAB2BGR)
        stamp = rospy.Time.from_sec(time.time())
        self.cam_info.header.stamp=stamp
        self.cam_info_raw.publish(self.cam_info)
        pub_im=self.bridge.cv2_to_imgmsg(imagen_cv)
        pub_im.height=imagen.height/2
        pub_im.width=imagen.width/2
        pub_im.step=imagen.step/2
        pub_im.encoding=imagen.encoding
        pub_im.header.frame_id='rgb'
        pub_im.header.stamp=stamp
        self.cam_pub.publish(pub_im)
    
    def depth_callback(self, imagen):
        imagen_cv = self.bridge.imgmsg_to_cv2(imagen, imagen.encoding)
        imagen_cv = cv2.resize(imagen_cv, (640/2,480/2))
        stamp = rospy.Time.from_sec(time.time())
        self.cam_info1.header.stamp=stamp
        self.cam_info_depth.publish(self.cam_info1)
        pub_im=self.bridge.cv2_to_imgmsg(imagen_cv)
        pub_im.height=imagen_cv.shape[0]
        pub_im.width=imagen_cv.shape[1]
        pub_im.step=imagen_cv.strides[0]
        pub_im.encoding=imagen.encoding
        pub_im.header.frame_id='depth'
        pub_im.header.stamp=stamp
        self.depth_pub.publish(pub_im)
if __name__ == '__main__':
    rospy.init_node('rgbd_crop')
    
    rgbd_crop()
    rospy.spin()
