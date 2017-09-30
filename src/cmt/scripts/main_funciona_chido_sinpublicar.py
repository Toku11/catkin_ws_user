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
    #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/ocam/image_raw",Image,self.callback,queue_size=1)
    #self.image_sub = rospy.Subscriber("/usb2_cam/image_rect_color",Image,self.callback)
    self.CRT = CMT.CMT()
    


    self.CRT.estimate_scale = 'estimate_scale'
    self.CRT.estimate_rotation = 'estimate_rotation'
    self.pause_time = 10
    ###########################Primer Region
    
    im0 = cv2.imread('./frame_cap.jpg', flags=cv2.IMREAD_GRAYSCALE)
    #im0 = cv2.imread('./frame_cap3.jpg', flags=cv2.IMREAD_GRAYSCALE)#flags=cv2.IMREAD_GRAYSCALE)
    #im0 = cv2.imread('/home/sergio/catikin_ws_user/src/cmt/scripts/frame.jpg', flags=cv2.IMREAD_COLOR)
    #im0 = cv2.imread('/home/sergio/catikin_ws_user/src/cmt/scripts/frame.jpg',flags=cv2.IMREAD_GRAYSCALE)
    #cv2.imshow('im0', im0)
    #im_gray0 = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
    im_gray0=im0
    im_draw = np.copy(im0)
    print('Selecciona Primer Region')
    #tl=(84, 55) #talvez
    #br=(557, 307#tal vez
    tl=(68, 68) 
    br=(544, 316)
    #tl=(35,35)
    #bl=(605,325)
    #(tl, br) = util.get_rect(im_draw)
    print('usando %i, %i como init bb', tl, br)
    self.CRT.initialise(im_gray0, tl, br)
    self.frame = 1
    self.conta=0

  def callback(self,data):
    try:
        tic = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  
        # Read image
        #status, im = cap.read()
        #im=cv_image
        #im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        im_draw = np.copy(cv_image)

        
        self.CRT.process_frame(im_gray)
        

        # Display results

        # Draw updated estimate
        if self.CRT.has_result:
          cv2.line(im_draw, self.CRT.tl, self.CRT.tr, (255, 0, 0), 4)
          cv2.line(im_draw, self.CRT.tr, self.CRT.br, (255, 0, 0), 4)
          cv2.line(im_draw, self.CRT.br, self.CRT.bl, (255, 0, 0), 4)
          cv2.line(im_draw, self.CRT.bl, self.CRT.tl, (255, 0, 0), 4)
          font = cv2.FONT_HERSHEY_SIMPLEX
          cv2.putText(im_draw,'1',(self.CRT.tl[0],self.CRT.tl[1]), font, 2, (200,255,155), 7, cv2.CV_AA)
          cv2.putText(im_draw,'2',(self.CRT.tr[0],self.CRT.tr[1]), font, 2, (200,255,155), 7, cv2.CV_AA)


          p1=[self.CRT.tl[0],self.CRT.tl[1],1]
          p2=[self.CRT.tr[0],self.CRT.tr[1],1]
          p3=[self.CRT.bl[0], self.CRT.bl[1],1]
          p4=[self.CRT.br[0], self.CRT.br[1],1]

          MCI=np.array([p1,p2,p3,p4])
          MCR=np.array([[-3,1.5,0,1],[3,1.5,0,1],[-3,-1.5,0,1],[3,-1.5,0,1]])
          vz=np.array([0,0,0,0])
          a=0
          #MT=np.empty(8,12)
          #MT=np.empty(shape=[8,12])
          MT= np.empty((8,12))
          #MT
          for j in xrange(4):#cantidad de puntos
            v1=(MCI[j][2]*MCR[j])*-1
            v2=(MCI[j][1]*MCR[j])
            v3=(MCI[j][2]*MCR[j])
            v4=(MCI[j][0]*MCR[j])*-1
            MT[j*2]=np.concatenate((vz,v1,v2))
            MT[(j*2)+1]=np.concatenate((v3,vz,v4))
            
          U, s, V = np.linalg.svd(MT, full_matrices=True)
          Tt=V[11]    
          T=np.reshape(Tt,[3,4])
          H=np.array([[T[0][0],T[0][1],T[0][3]],
                    [T[1][0],T[1][1],T[1][3]],
                    [T[2][0],T[2][1],T[2][3]]
                    ])
          Mb=np.array(H[0:3,0:2])
          nU, ns, nV = np.linalg.svd(Mb, full_matrices=True)
          aux=nU[0:3,0:2]
          Tb=np.dot(aux,nV)
          naux=np.cross(Tb[0:3,0],Tb[0:3,1])

          T=np.append(Tb,naux.reshape([3,1]),axis=1)#matriz rotacion
          deter=np.linalg.det(T)
          if deter<0:
            R=T*np.array([[-1,0,0],[0,-1,0],[0,0,1]])
          else:
            R=T
          lam=np.trace(np.dot(np.transpose(Tb),Mb))/np.trace(np.dot(np.transpose(Mb),Mb))
          w=(np.dot(np.dot(np.transpose(T),H),np.array([[0],[0],[lam]])))
          if w[2]<1:
            w[2]=w[2]*-1
          
          







        #util.draw_keypoints(self.CRT.tracked_keypoints, im_draw, (255, 255, 255))
        # this is from simplescale
        #util.draw_keypoints(self.CRT.votes[:, :2], im_draw)  # blue
        #util.draw_keypoints(self.CRT.outliers[:, :2], im_draw, (0, 0, 255))


        
        #cv2.imshow('main', im_draw)

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
        #print(self.CRT.tl[0])
        #print(self.CRT.tl[1])
        #print(self.CRT.tl.shape)
        

        try:
          #self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_draw, "bgr8"))
          toc = time.time()
          #print(1000 * (toc - tic))
          print(w[0],",",w[1],",",w[2],",",1000 * (toc - tic))
          #print('P',w[1])
          #print('P',w[2])
        except CvBridgeError as e:
          print(e)
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
