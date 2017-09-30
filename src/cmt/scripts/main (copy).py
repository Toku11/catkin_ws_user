#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('cmt')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import empty, nan
import os
import sys
import time
import CMT
import numpy as np
import util




class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/usb2_cam/image_rect_color",Image,self.callback)
    self.CRT = CMT.CMT()
    self.CNT = CMT.CMT()
    self.CST = CMT.CMT()

    self.CRT = CMT.CMT()
    self.CNT = CMT.CMT()
    self.CST = CMT.CMT()


    self.CRT.estimate_scale = 'estimate_scale'
    self.CRT.estimate_rotation = 'estimate_rotation'
    self.CNT.estimate_scale = 'estimate_scale'
    self.CNT.estimate_rotation = 'estimate_rotation'
    self.CST.estimate_scale = 'estimate_scale'
    self.CST.estimate_rotation = 'estimate_rotation'
    self.pause_time = 10
    ###########################Primer Region
    
    
    im0 = cv2.imread('~/catikin_ws_c/src/cmt/scripts/imas.jpg', flags=cv2.IMREAD_COLOR)
    if im0:
      print(im0.shape)
    im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
    im_draw = np.copy(im0)
    print('Selecciona Primer Region')
    (tl, br) = util.get_rect(im_draw)
    print('using %i, %i as init bb', tl, br)
    self.CRT.initialise(im_gray0, tl, br)
    ###########################Segunda Region
    print('Selecciona Segunda Region')
    (tl, br) = util.get_rect(im_draw)
    print('using %i, %i as init bb', tl, br)
    self.CNT.initialise(im_gray0, tl, br)
    ############################Tercer Region
    print('Selecciona Tercer Region')
    (tl, br) = util.get_rect(im_draw)
    print('using %i, %i as init bb', tl, br)
    self.CST.initialise(im_gray0, tl, br)
    self.frame = 1

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Read image
    #status, im = cap.read()
    #im=cv_image
    #im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    im_draw = np.copy(cv_image)

    tic = time.time()
    self.CRT.process_frame(im_gray)
    self.CNT.process_frame(im_gray)
    self.CST.process_frame(im_gray)
    toc = time.time()

    # Display results

    # Draw updated estimate
    if self.CRT.has_result:
      cv2.line(im_draw, self.CRT.tl, self.CRT.tr, (255, 0, 0), 4)
      cv2.line(im_draw, self.CRT.tr, self.CRT.br, (255, 0, 0), 4)
      cv2.line(im_draw, self.CRT.br, self.CRT.bl, (255, 0, 0), 4)
      cv2.line(im_draw, self.CRT.bl, self.CRT.tl, (255, 0, 0), 4)

    if self.CNT.has_result:
      cv2.line(im_draw, self.CNT.tl, self.CNT.tr, (255, 255, 0), 4)
      cv2.line(im_draw, self.CNT.tr, self.CNT.br, (255, 255, 0), 4)
      cv2.line(im_draw, self.CNT.br, self.CNT.bl, (255, 255, 0), 4)
      cv2.line(im_draw, self.CNT.bl, self.CNT.tl, (255, 255, 0), 4) 

    if self.CST.has_result:
      cv2.line(im_draw, self.CST.tl, self.CST.tr, (0, 255, 0), 4)
      cv2.line(im_draw, self.CST.tr, self.CST.br, (0, 255, 0), 4)
      cv2.line(im_draw, self.CST.br, self.CST.bl, (0, 255, 0), 4)
      cv2.line(im_draw, self.CST.bl, self.CST.tl, (0, 255, 0), 4) 

    util.draw_keypoints(self.CRT.tracked_keypoints, im_draw, (255, 255, 255))
    # this is from simplescale
    util.draw_keypoints(self.CRT.votes[:, :2], im_draw)  # blue
    util.draw_keypoints(self.CRT.outliers[:, :2], im_draw, (0, 0, 255))


    
    cv2.imshow('main', im_draw)

      # Check key input
    #k = cv2.waitKey(self.pause_time)
    #key = chr(k & 255)
    #if key == 'q':
    #  break
    #if key == 'd':
    #  import ipdb; ipdb.set_trace()

    # Remember image
    im_prev = im_gray

    # Advance frame number
    self.frame += 1

    print('{5:04d}: center: {0:.2f},{1:.2f} scale: {2:.2f}, active: {3:03d}, {4:04.0f}ms'.format(self.CRT.center[0], self.CRT.center[1], self.CRT.scale_estimate, self.CRT.active_keypoints.shape[0], 1000 * (toc - tic), self.frame))

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
