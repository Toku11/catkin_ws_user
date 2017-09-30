#!/usr/bin/python
# -*- coding: latin-1 -*-
from __future__ import print_function
import roslib
roslib.load_manifest('callback_test')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math


    
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback)
    #self.pub_ang = rospy.Publisher("/angle_lane", Float32, queue_size=10)
    

  def callback(self,data):
    try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

        #_, frame = cap.read()    
    frame = cv2.resize(frame, (160,120))
    print('imagen r')
    self.pub_ang.publish((np.abs(ang)-90)*np.sign(ang))
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  r = rospy.Rate(2)
  while not rospy.is_shutdown():
      cv2.waitKey(1)
      r.sleep()
  #try:
  #  rospy.spin()
  #except KeyboardInterrupt:
  #  print("Shutting down")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    main(sys.argv)
