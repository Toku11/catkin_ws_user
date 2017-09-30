#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function
import roslib
roslib.load_manifest('lane_detect_python')
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import sys
from scipy import ndimage

ransac_iterations = 40  # number of iterations
ransac_threshold = 4# threshold
ransac_ratio = .6 # ratio of inliers required to assert
                       # that a model fits well to data
n_inputs = 1
ang=0
contador=0
n_outputs = 1



def faster_bradley_threshold(image, threshold=82, window_r=5):
    percentage = threshold / 100.
    window_diam = 2*window_r + 1
    # convert image to numpy array of grayscale values
    img = image.astype(np.float) # float for mean precision 
    # matrix of local means with scipy
    means = ndimage.uniform_filter(img, window_diam)
    # result: 0 for entry less than percentage*mean, 255 otherwise 
    height, width = img.shape[:2]
    result = np.ones((height,width), np.uint8)*255   # initially all 0
    result[img >= percentage * means] = 0      # numpy magic :)
    # convert back to PIL image
    return (result)
    
    
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
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/crop/rgb/image_raw",Image,self.callback,queue_size=1)
        self.pub_ang = rospy.Publisher("/angle_lane", Float32, queue_size=1)
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
        self.conta=0
        self.conta_obs=0
        self.contador=0
        self.ang=0
        self.filter_ang=0
        self.contador1=0
        self.car=0
    
    def callback(self,data):
        
        
        try:
          #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
          frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
    # Take each frame
        self.conta=self.conta+1
        if(self.conta%2==0):
            frame = cv2.resize(frame, (160,120))
            (rows,cols,channels) = frame.shape
            tam_crop_g=42#crop para carretera
            tam_crop=tam_crop_g/2
            tam_crop2=0#crop para ancho
            crop_img = frame[rows-tam_crop_g:rows, tam_crop2:cols-tam_crop2]
            offsetx=45+20#centrar en imagen vista aérea    
            
            gray = cv2.cvtColor(crop_img,cv2.COLOR_BGR2GRAY)      
            (rows3,cols3) = gray.shape
            correct=39#que tan alargada de arriba se quiere corregir
            elong=100#que tanto alargado se quiere    
            way=np.array([[correct,0],[0,tam_crop],[cols3,tam_crop],[cols3-correct,0]],dtype="float32")#chidocam_coche_res
            nwy=np.array([[offsetx+0,0],[offsetx+0,tam_crop+elong],[offsetx+ cols3,tam_crop+elong],[offsetx+cols3,0]],dtype="float32")#chidocam_coche_res    
            M=cv2.getPerspectiveTransform(way,nwy)
            maxIntensity=255.0
            phi=1.2
            theta=2
            gray3=(maxIntensity/phi)*(gray/(maxIntensity/theta))**7
            gray3[np.where(gray3<0)]=0
            gray3[np.where(gray3>255)]=255
            gray3 =(np.uint8(gray3))    
            warped=cv2.warpPerspective(gray3,M,(cols3+(correct*2)+50,rows3+elong+10))
            warped=cv2.resize(warped,(200,105)) 


            ret, warped = cv2.threshold(warped,70,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
      
            #warped = np.uint8(np.absolute(cv2.Sobel(warped,cv2.CV_64F,1,0,ksize=3)))
            warped=cv2.Canny(warped,50,150,apertureSize=3)
            #cv2.imshow('tst',warped)
            cv2.line(warped,(0,14),(57,105),(0,0,0),5)    
            #cv2.line(sobelx,(0,77),(21,96),(255,255,255),2)    
            cv2.line(warped,(200,14),(143,105),(0,0,0),5)
            (rows2,cols2)=warped.shape
           # warped=faster_bradley_threshold((warped))
            num_lin=50
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
                    if new_p[1]==aux[1] and (new_p[0]<aux[0]+10 and new_p[0]>aux[0]-10):###holgura para x
                        valor.append(new_p[0])
                    else: #new_p[1]>aux[1]:
                        if len(valor)>0:## promedio para un solo punto
                            for i in range(0,len(valor)):
                                suma=suma+valor[i]
                            prom=suma/len(valor)
                            nue=(prom,aux[1])
                            cv2.circle(warped,nue, 2, (255,255,255),2)
                            lista.append(nue)
                        suma=0
                        valor=[]
                        valor.append(new_p[0])
                    aux=new_p
            ls=np.asarray(lista)
            ordp=[]
            ordp.append([])
            ##################################################
           # y=ls[:,1]
           # x=ls[:,0] 
            centro=round(cols2/2)#centro de la imágen del cochesito
            
            #####################################################3
            ## SE GENERAN LISTAS DE CADA LINEA
            for val in ls:
                if len(ordp[0])==0:#lista nueva no inicializad
                    ordp[0].append(val)
                else:
                    len(ordp)
                    flag=False
                    for i in range(0,len(ordp)):
                        ## listas distancia en X y en Y
                        if np.abs(val[0]-ordp[i][len(ordp[i])-1][0])<8 and np.abs(val[1]-ordp[i][len(ordp[i])-1][1])<=(rws[0]-rws[5]):
                            ordp[i].append(val)
                            flag=True
                            break
                        elif i==len(ordp)-1 and flag==False:
                            ordp.append([])
                            ordp[i+1].append(val)
            ############################################################
            #################################################################3                    
            ######SE DESCARTAN LINEAS
            tam_carril=44##Es el tamaño de carril a detectar y a dibujar si o existe otro
            value=[]
            holg=15
            flg=True
            flg2=True
            BANDERA=True
            n=0                  
            for x in range(0,len(ordp)):
                if (ordp[x][0][1]>60) and (len(ordp[x])>8):##LIMITE Y barrido y LIMITE DE TAMAÑO de linea
                    value.append([x,(ordp[x][1][0]-centro),len(ordp[x]),ordp[x][1][0]])
                    if np.size(value)==0:
                        BANDERA=False   

            if BANDERA==True:
                value=np.asarray(sorted(value,key=lambda x:x[1]))
                ind1=np.argmin(abs(value[:,1]))##lugar del minimo al centro  
                indice=int(value[ind1][0]) ##lugar en ordp del minimo al centro
              ###############################################
              #########################################################################3
                ## Se detecta el carril     
            
            ##############CAMBIOS PARA ROOOOSSSS####################
            ##carril derecho
                if value[ind1][1]<=0: #linea izquierda y posible carril
                    value_neg=value[np.where(value[:,1]>0)]#borrar valor negativo
                    
                    if len(value_neg)>0:## que no este vacio el arreglo 
                        value_neg[:,1]=abs((value_neg[:,1])-value[ind1][1])##distancia entre dos puntos
                        value_neg=value_neg[np.where(np.all([value_neg[:,1]>tam_carril-holg,value_neg[:,1]<tam_carril+holg],0)==True)]##restriccion tamaño carril
                        flg=True

                    else: ##no existe otro carril         
                        xx=np.array([value[ind1][3]+tam_carril,ordp[indice][1][1]])
                        xx1=np.asarray(ordp[indice])
                        value_neg=np.array([[0,tam_carril]])
                        kk=0
                        flg=False
                        self.car=0
                       # print('Generando linea: carril derecho')
                        
                    if  len(value_neg)>0 and flg==True:
                        kk=np.argmin(value_neg[:,1])
                        g=value_neg[kk,0]
                        nueva=ordp[int(g)]
                        xx1=np.asarray(ordp[indice])
                        xx=np.asarray(nueva)
                        #print('Estoy en carril derecho') 
                        self.car=1
                    elif flg==True:
                        xx=np.array([value[ind1][3]+tam_carril,ordp[indice][1][1]])
                        xx1=np.asarray(ordp[indice])
                        value_neg=np.array([[0,tam_carril]])
                        kk=0
                        self.car=0
                        #print('Generando nuevo carril')
            #####################################################
                        ###########RANSAC################
            #
                    if len(xx1)>=len(xx):
                        change=1
                    else:
                        change=-1
                        
            #############################################################
            #        
            #  #########################################################################3
            #    ## Se detecta el carril    
                elif value[ind1][1]>0: #linea derecha y posible carril  
                    value_neg=value[np.where(value[:,1]<0)]#borrar valor positivo    
                    value_pos=np.delete(value,ind1,0)
                    value_pos=value[np.where(value_pos[:,1]>0)]
            
                    if len(value_neg)>0:## que no este vacio el arreglo 
                        value_neg[:,1]=abs((value_neg[:,1])-value[ind1][1])##distancia entre dos puntos
                        value_neg=value_neg[np.where(np.all([value_neg[:,1]>tam_carril-holg,value_neg[:,1]<tam_carril+holg],0)==True)]##restriccion tamaño carril
                        flg=True
                    
                        #print('existe carril a mi derecha')
                    else:## no existe otro carril 
                        xx=np.array([value[ind1][3]-tam_carril,ordp[indice][1][1]])
                        xx1=np.asarray(ordp[indice])
                        value_neg=np.array([[0,tam_carril]])
                        flg=False
                        kk=0
                        self.car=0
                        #print('Generando linea: carril izquierdo')
                    if len(value_neg)>0 and flg==True:
                        kk=np.argmin(value_neg[:,1])
                        g=value_neg[kk,0]
                        nueva=ordp[int(g)]
                        xx1=np.asarray(ordp[indice])
                        xx=np.asarray(nueva)
                        #print('Estoy en carril izquierdo')
                        self.car=1
                    elif flg==True:
                        xx=np.array([value[ind1][3]-tam_carril,ordp[indice][1][1]])
                        xx1=np.asarray(ordp[indice])
                        value_neg=np.array([[0,tam_carril]])
                        kk=0
                        self.car=0
                        #print('Generando nuevo carril')
            #####################################################
            #            ###########RANSAC################
                    if len(xx1)>=len(xx):
                        change=-1
                    else:
                        change=1
            #############################################################
            #            
                else:
                    print('no existe un carril')
            
                #print(self.ang)
                
        
                
        
                if(self.contador>7) or (self.contador1>2):
                    xx=np.array([value[ind1][3]+tam_carril*np.sign(self.filter_ang),ordp[indice][1][1]])
                    xx1=np.asarray(ordp[indice])
                    kk=0
                    change=np.sign(self.filter_ang)
                    
                if len(xx1)>=len(xx):
                    line_fol=xx1
                else:
                    line_fol=xx
            #########################################################################
            #TERMINAN CAMBIOS PARA ROOOOOSSS
            
            
               
               
               
               
               
                    
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
                        
                        # check whether it's an inlier or not
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
                    
                    # we are done in case we have enough inliers
                    if num > n_samples*ransac_ratio:
                      #  print 'The model is found !'
                        break
            
                xmin=0#min(line_fol[:,0])
                xmax=250#max(line_fol[:,0])
                ss=-(np.cos(theta))/(np.sin(theta))
                #c=rho/(np.sin(theta))
                d=(rho+value_neg[kk,1]/2*change)/np.sin(theta)
                cv2.line(warped,(xmin,long(xmin*ss+d)),(xmax,long(xmax*ss+d)),(255,0,0),2)  
                #cv2.line(warped,(int(centro),0),(int(centro),int(rows2)),(255,255,0),2)
                #cv2.line(warped,(int(centro+44),0),(int(centro+44),int(rows2)),(255,255,0),2)
                ### Seguimiento de linea
                
                y_seg=rows2-27
                x_seg=(y_seg-d)/ss##coordenada x de ransac centro de carril 
                self.ang=np.rad2deg(np.arctan((y_seg-rows2)/(x_seg-centro)))#angulo con respecto a horizontal
                self.ang=(np.abs(self.ang)-90)*np.sign(self.ang)        
               # print('angulo=',ang)
                cv2.line(warped,(int(centro),int(rows2)),(int(x_seg),int(y_seg)),(255,0,0),2)
                alpha=0.8              
            
            self.filter_ang=self.filter_ang*alpha+(1-alpha)*self.ang
            if(abs(self.ang)>30):
                self.contador+=1
            else:
                self.contador=0
            if (abs(self.filter_ang)>20):
                self.contador1+=1
            else:
                self.contador1=0
            try:
                if self.car==1:
                    self.pub_ang.publish(self.filter_ang)
              #self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped, "mono8"))
            except CvBridgeError as e:
                print(e)

        
        
        
        #cv2.imshow('gray',gray3)
        #cv2.imshow('warped',warped)
        #cv2.imshow('warped2',warped2)
    
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
