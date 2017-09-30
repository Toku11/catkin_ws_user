#!/usr/bin/python
# -*- coding: latin-1 -*-
from __future__ import print_function
import roslib
roslib.load_manifest('lane_detect_python')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy.polynomial as P
import math

ransac_iterations = 20  # number of iterations
ransac_threshold = 2# threshold
ransac_ratio = 0.8 # ratio of inliers required to assert
                        # that a model fits well to data
n_inputs = 1
n_outputs = 1



def find_line_model(points):
 
    # [WARNING] vertical and horizontal lines should be treated differently
    #           here we just add some noise to avoid division by zero
 
    # find a line model for these points
    m = (points[1,1] - points[0,1]) / (points[1,0] - points[0,0]+ sys.float_info.epsilon)  # slope (gradient) of the line
    c = points[1,1] - m * points[1,0]    
           # y-intercept of the line
    return m, c, points
    
def find_intercept_point(m, c, x0, y0):
    # intersection point with the model
    x = (x0 + m*y0 - m*c)/(1 + m**2)
    y = (m*x0 + (m**2)*y0 - (m**2)*c)/(1 + m**2) + c
 
    return x, y      
    
    
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback)
    self.pub_ang = rospy.Publisher("/angle_lane", Float32, queue_size=10)
    
    
    
  
    

    
    
    

  def callback(self,data):
    try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)




    frame = cv2.resize(frame, (160,120))
    (rows,cols,channels) = frame.shape
    tam_crop_g=40
    tam_crop=15
    tam_crop2=0
    crop_img = frame[rows-tam_crop_g:rows-10, tam_crop2:cols-tam_crop2]
    (row,col,v) = crop_img.shape
    #print(row)
    #print(col)
    crop_img = cv2.resize(crop_img, (80,15)) 
    offsetx=15    
    #gray2 = cv2.cvtColor(resized_image,cv2.COLOR_BGR2GRAY)  
    gray = cv2.cvtColor(crop_img,cv2.COLOR_BGR2GRAY)  
    (rows3,cols3) = gray.shape

    correct=10
    elong=80    
    way=np.array([[correct,0],[0,tam_crop],[cols3,tam_crop],[cols3-correct,0]],dtype="float32")#chidocam_coche_res
    nwy=np.array([[offsetx+0,0],[offsetx+0,tam_crop+elong],[offsetx+ cols3,tam_crop+elong],[offsetx+cols3,0]],dtype="float32")#chidocam_coche_res  
    M=cv2.getPerspectiveTransform(way,nwy)
    maxIntensity=255.0
    phi=.45#.52
    theta=1.9#3.7 
    gray3=(maxIntensity/phi)*(gray/(maxIntensity/theta))**8
    gray3[np.where(gray3<0)]=0
    gray3[np.where(gray3>255)]=255
    gray3 =(np.uint8(gray3))    
    warped=cv2.warpPerspective(gray3,M,(cols3+160,rows3+elong-18))
    #warped=cv2.warpPerspective(gray3,M,(cols3+80,rows3+elong-60))
    #warped2=cv2.warpPerspective(gray3,M,(cols3+200,rows3+elong-60))
    #warped[np.where(warped>90)]=255#hacemos blancos
    #warped[np.where(warped<70)]=0#hacemos negros
    
    #warped2=cv2.warpPerspective(crop_img,M,(cols3-50,rows3+elong))#alargada a color    
        
    ret, thresh = cv2.threshold(warped,50,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    
    sobelx = cv2.Sobel(thresh,cv2.CV_64F,1,0,ksize=3)#derivada para imágen
    #sobelx=cv2.Canny(warped,50,150,apertureSize = 3)
    abs_sobel=np.absolute(sobelx)
    sobelx=np.uint8(abs_sobel)
    #warped=cv2.resize(sobelx,(220,250))
    cv2.line(sobelx,(0,0),(70,172),(0,0,0),8)    
    cv2.line(sobelx,(250,172),(319,0),(0,0,0),8)    
    warped=sobelx    
    (rows2,cols2)=warped.shape
    num_lin=30
    rws=np.round(np.linspace(rows2-1,0,num_lin))#dividimos en diferentes regiones        
    arreglo=np.where(warped[[rws],:])#deteccion de puntos blanco en cada linea en que se dividió

    pr=0
    valor=list()
    lista=list()
    aux=(0,0)
    suma=0
    #############################################
    #arreglo de puntos donde se descartan puntos cercanos se promedia para obtener un solo puntos
    init_flag=False 
    for idx in arreglo[1]:
        new_p=(arreglo[2][pr],int(rws[idx]))    
        pr=pr+1 
        if init_flag==False:
            valor.append(new_p[0])
            aux=new_p
            init_flag=True
        else: 
            if new_p[1]==aux[1] and (new_p[0]<aux[0]+20 and new_p[0]>aux[0]-20):
                valor.append(new_p[0])
            else: #new_p[1]>aux[1]:
                if len(valor)>0:## promedio para un solo punto
                    for i in range(0,len(valor)):
                        suma=suma+valor[i]
                    prom=suma/len(valor)
                    nue=(prom,aux[1])
                    #cv2.circle(warped,nue, 1, (255,255,255),2)
                    lista.append(nue)
                suma=0
                valor=[]
                valor.append(new_p[0])
            aux=new_p
    ls=np.asarray(lista)
    ordp=[]
    ordp.append([])
    ################ ##################################
    y=ls[:,1]
    x=ls[:,0] 
    centro=round(cols2/2)-0#centro de la imágen del cochesito
    
    #####################################################3
    ## SE GENERAN LISTAS DE CADA LINEA
    for val in ls:
        if len(ordp[0])==0:#lista nueva no inicializad
            ordp[0].append(val)
        else:
            len(ordp)
            flag=False
            for i in range(0,len(ordp)):
                if np.abs(val[0]-ordp[i][len(ordp[i])-1][0])<12 and np.abs(val[1]-ordp[i][len(ordp[i])-1][1])<=(rws[0]-rws[3]):
                    ordp[i].append(val)
                    flag=True
                    break
                elif i==len(ordp)-1 and flag==False:
                    ordp.append([])
                    ordp[i+1].append(val)
    ############################################################
    #################################################################3                    
    ######SE DESCARTAN LINEAS
    bandera=True
    value=[]                    
    for x in range(0,len(ordp)):
        if (ordp[x][0][1]>0) and (len(ordp[x])>3):##LIMITE Y y LIMITE DE TAMAÑO
            value.append([x,(ordp[x][1][0]-centro),len(ordp[x]),ordp[x][1][0]])
    
    #value=sorted(value,key=lambda x:x[3]) ##acomodado por y  1
    value=np.asarray(value)
    ind1=np.argmin(abs(value[:,1]))##lugar del minimo al centro
    
    
    indice=int(value[ind1][0]) ##lugar en ordp del minimo al centro
    #print(indice)
  #########################################################################3
    ## Se detecta el carril    
    if value[ind1][1]<=0 and any(x>0 for x in value[:,1])==True: #linea izquierda y posible carril
        valor=value[ind1][1]##se guarda valor de linea a seguir
        value_neg=value[np.where(value[:,1]>0)]#borrar valor negativo
        if len(value_neg>0):## que no este vacio el arreglo
            value_neg[:,1]=abs((value_neg[:,1])-valor)##distancia entre dos puntos
            value_neg=value_neg[np.where(np.all([value_neg[:,1]>20,value_neg[:,1]<100],0)==True)]##restriccion tamaño carril
            if len(value_neg)>0:
                k=np.argmax(value_neg[:,1])
                g=value_neg[k,0]
                nueva=ordp[int(g)]
                xx1=np.asarray(ordp[indice])
                xx=np.asarray(nueva)
                bandera=False
#####################################################
                ###########RANSAC################
                if len(xx1)>=len(xx):
                    line_fol=xx1
                    change=1
                else:
                    line_fol=xx
                    change=-1
############################################################
               
                1
                
    elif value[ind1][1]>0 and any(x<0 for x in value[:,1])==True:
        valor=value[ind1][1]
        value_neg=value[np.where(value[:,1]<0)]
        if len(value_neg>0):
            value_neg[:,1]=abs((value_neg[:,1])-valor)
            value_neg=value_neg[np.where(np.all([value_neg[:,1]>20,value_neg[:,1]<100],0)==True)]
            ##entre linea cont y punteada hay 70 y entre dos continuas 152
            if len(value_neg)>0:
                bandera=False
                k=np.argmax(value_neg[:,1])#[0]))1
                g=value_neg[k,0]
                nueva=ordp[int(g)]
                xx1=np.asarray(ordp[indice])
                xx=np.asarray(nueva)

#####################################################
                ###########RANSAC################
                if len(xx1)>=len(xx):
                    line_fol=xx1
                    change=-1
                else:
                    line_fol=xx
                    change=1
############################################################
                
                
                    
    
    else:
        print('posiblemente no estas en carril')
        bandera=True
    
    if bandera==False:


        x_n=np.array([line_fol[:,0]]).T
        y_n=np.array([line_fol[:,1]]).T#[abs(xx1[:,1]-rws[0])]).T
        data =np.hstack( (x_n,y_n) )
        ratio = 0.
        model_m = 0.
        model_c = 0.
        n_samples=len(x_n)
     
    # perform RANSAC iterations
        for it in range(ransac_iterations): 
     
            # pick up two random points
            n = 2
     
            all_indices = np.arange(x_n.shape[0])
            np.random.shuffle(all_indices)
            
            indices_1 = all_indices[:n]
            indices_2 = all_indices[n:]
     
            maybe_points = data[indices_1,:]
            test_points = data[indices_2,:]
     
            # find a line model for these points
            m, c,points = find_line_model(maybe_points)
            #print(c)
            x_list = []
            y_list = []
            num = 0
     
            # find orthogonal lines to the model for all testing points
            for ind in range(test_points.shape[0]):
     
                x0 = test_points[ind,0]
                y0 = test_points[ind,1]
     
                # find an intercept point of the model with a normal from point (x0,y0)
                x1, y1 = find_intercept_point(m, c, x0, y0)
                
                # distance from point to the model
                dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
                
                # check whether it's an20 inlier or not
                if dist < ransac_threshold:
                    x_list.append(x0)
                    y_list.append(y0)
                    num += 1
                    
            x_inliers = np.array(x_list)
            y_inliers = np.array(y_list)
     
            # in case a new model is better - cache it
            if num/float(n_samples) > ratio:
                ratio = num/float(n_samples)
                model_m = m
                model_c = c
                x_final=points[1,0]
                y_final=points[1,1]
                theta=np.arctan2(points[1,1]-points[0,1],points[1,0]-points[0,0])       
                theta=theta-np.pi/2
                if np.abs(np.rad2deg(theta))>90:
                    theta=theta+np.pi
                #print('theta',np.rad2deg(theta))
                rho=points[1,0]*np.cos(theta)+points[1,1]*np.sin(theta)
            
            # we are done in case we have enough inliers(
            if num > n_samples*ransac_ratio:
                #print 'The model is found !'
                break
    
        xmin=0#min(line_fol[:,0])
        xmax=250#max(line_fol[:,0])
        ss=-(np.cos(theta))/(np.sin(theta))
        c=rho/(np.sin(theta))
        d=(rho+value_neg[k,1]/2*change)/np.sin(theta)
        cv2.line(warped,(xmin,long(xmin*ss+d)),(xmax,long(xmax*ss+d)),(255,0,0),2)  
       # cv2.line(warped,(int(centro),0),(int(centro),int(rows2)),(255,255,0),2)
        
        ### Seguimiento de linea
        
        y_seg=rows2-58
        x_seg=(y_seg-d)/ss##coordenada x de ransac centro de carril 
        ang=np.rad2deg(np.arctan((y_seg-rows2)/(x_seg-centro)))#angulo con respecto a horizontal
        #print('angulo=',(90-np.abs(ang))*np.sign(ang))
        self.pub_ang.publish((np.abs(ang)-90)*np.sign(ang))
        
        cv2.line(warped,(int(centro),int(rows2)),(int(x_seg),int(y_seg)),(255,0,0),2)



    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped, "mono8"))
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
