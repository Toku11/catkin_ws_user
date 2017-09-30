#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>
class parking
{
public:
    
    ros::Publisher steering_publisher;
	parking(ros::NodeHandle nh)
	{
	    //light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);

        steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
        //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("parking_steering_cmd"), 1);
        //speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("parking_speed_cmd"), 1);
		subScan_ = nh.subscribe("scan", 1, &parking::scanCallback,this);
	    //head_subscriber = nh.subscribe("model_car/yaw", 1, &parking::headCallback,this);
		subTwist_ = nh.subscribe("model_car/yaw2",1,&parking::headingCallback,this); 
		odometry_sub = nh.subscribe("odom",1,&parking::ReceiveOdometry,this);
		//pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		angle_front=10;
		angle_back=10;
		break_distance=0.85;
		init_yaw=false;
		init_park=false;
		flag_p=false;
		flag_p1=false;
		flag_p2=false;
		flag_p3=false;
        flag_np=false;
        flag_can_start=false;
        flag_motor_has_started=false;
		parked=false;
		p_i=0;
		c_d=0;
        c_s=0;
        found_park=false;
        init_search_place=false;
        obj1=false,obj2=false;

        speed_command.data=0;
        steering_command.data=96;//defelccion derecha
        speed_publisher.publish(speed_command);//detenemos
        steering_publisher.publish(steering_command);
	}
	~parking(){}

    void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg)
 	//void ReceiveOdometry(ConstPtrst nav_msgs::Odometry& msg)
    {
    	if (init_park==false&&init_search_place==true){
            speed_command.data=0;
            steering_command.data=96;//defelccion derecha
            speed_publisher.publish(speed_command);//detenemos
            steering_publisher.publish(steering_command);
            ROS_INFO("INIcializado");
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;
            theta_ini=-1*msg->twist.twist.linear.y*3.1416/180;
    	    //norm=sqrt((px*px)+(py*py));
    		x_ini=px;
    		y_ini=py;
            ROS_INFO("thet %f",theta_ini*180/3.1416);
            ROS_INFO("pxini %f %f",x_ini,y_ini);
            spx=x_ini+3.5*sin(theta_ini);
            spy=y_ini+3.5*cos(theta_ini);
            ROS_INFO("pfini %f %f",spx,spy);
    		init_park=true;
            speed_command.data=-1000;//delante para buscar espacio de estacionamiento
            speed_publisher.publish(speed_command);//
            p_i=0;
            

    	}else{
           if(flag_can_start==false){ 
            		px=msg->pose.pose.position.x;
                    py=msg->pose.pose.position.y;
                    norm=sqrt((px*px)+(py*py));
                    err_x=spx-px;
                    err_y=spy-py;
                    if(err_x<.03 && err_x>-.03 && err_y<.03 && err_y>-.03){               
                       //llegue al lugar
                       speed_command.data=0;//delante para buscar espacio de estacionamiento
                       speed_publisher.publish(speed_command);//
                       flag_can_start=true;
                       x_ini=px;
                       y_ini=py;
                    }
                    //sp_yaw=atan2(err_y,err_x)*180/3.1416;
                    sp_yaw=-atan2(err_x,err_y)*180/3.1416;
                    yaw=msg->twist.twist.linear.y;
                    error_yaw=(sp_yaw-yaw);

                    if (error_yaw < -180.0 ){
                        error_yaw=error_yaw+360;
                    }else if (error_yaw > 180.0 ){
                        error_yaw=error_yaw-360;
                    }
                    
                    //////////////////////////////
                    if(error_yaw>89||error_yaw<-89){
                        direction=70;//atras
                        //speed_command.data=200;
                        //k_idx+=1;

                        error_yaw+=180;//agregamos offset

                        if (error_yaw < -180.0){
                            error_yaw=error_yaw+360;
                        }else if (error_yaw > 180.0 ){
                            error_yaw=error_yaw-360;
                        }


                    }else{
                        direction=-70;//adelante
                        //speed_command.data=velocity;
                    }
                    //////////////////////


                    if(direction<-30){//hacia delante
                        ctrl_yaw=96+3*(error_yaw);//+5.1*(error_yaw-last_error);//2*(error_yaw);
                    }
                    else if (direction>30){//atrás
                        
                        ctrl_yaw=96-2*(error_yaw);//2*(error_yaw);
                    }
                    last_error=error_yaw;
                    if(ctrl_yaw>180){
                                ctrl_yaw=180; 
                    }else if(ctrl_yaw<1){
                           ctrl_yaw=1; 
                    }
                    c_d++;
                    if(c_d%3==0){
                        steering_command.data=ctrl_yaw;//defelccion derecha
                        steering_publisher.publish(steering_command);
                        //ROS_INFO("n %f",norm);
                        //ROS_INFO("yawer %f,%f,%f",error_yaw,err_x,err_y);                   
                    }

           }else{
                    px=x_ini-msg->pose.pose.position.x;
                    py=y_ini-msg->pose.pose.position.y;
                    norm=sqrt((px*px)+(py*py));
            	    switch(p_i){
                    case 0:

                      if (flag_np==false){    
                          if(flag_can_start==true){//nos desplazamos a la posicion de parking inicial
                           
                           speed_command.data=0;//stop
                           ROS_INFO("case0"); 
                           ROS_INFO("norm %f",norm);  
                           steering_command.data=0;//defelccion derecha
                           speed_publisher.publish(speed_command);//detenemos
                           steering_publisher.publish(steering_command);
                           speed_publisher.publish(speed_command);//detenemos
                           
                           steering_publisher.publish(steering_command);
                           last_time = ros::Time::now(); 
                           flag_np=true;           
                          }
                      }else{
                          if((ros::Time::now()-last_time).toSec()>0.5){
                            x_ini=msg->pose.pose.position.x;                     
                            y_ini=msg->pose.pose.position.y;
                            norm=0;
                            p_i=1;                    
                          }
                      }    
                    break;    
            		case 1:
            		    if (flag_p1==false){
                            ROS_INFO("case1");
                            ROS_INFO("norm %f",norm);
            				flag_p1=true;
            				
            				speed_command.data=480;//reversa
            			    //steering_command.data=-80;//defelccion derecha
                            steering_command.data=0;//defelccion derecha

                            speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                            speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                            
            			}
                        newangu=((theta_ini*180/3.1416)+39);
                        if (newangu>180)
                             newangu=newangu-360;
                        else if (newangu<-180.0)
                         newangu=newangu+360;
            			if(norm>=0.45|| msg->twist.twist.linear.y>newangu){//0.28
                            printf("angu %f,%f\n",msg->twist.twist.linear.y,norm );
            			  p_i=2;
            		    }


            		break;
            		case 2:
            		    if (flag_p2==false){
                            ROS_INFO("case2");
                            ROS_INFO("norm %f",norm);
            				flag_p2=true;
            				last_time = ros::Time::now();    				
            			    //steering_command.data=98;//defelccion centro+
                            steering_command.data=180;//defelccion derecha
                            //speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                            //speed_publisher.publish(speed_command);//detenemos
                          steering_publisher.publish(steering_command);
                            
            			}
            			//if((ros::Time::now()-last_time).toSec()>1.4){
            				//x_p=msg->pose.pose.position.x;
            				//y_p=msg->pose.pose.position.y;
            			//	p_i=2;
            			//}
            			//steering_command.data=96;//defelccion derecha
            			//if(norm>=0.15){//0.28

            			if(norm>=0.16){//0.28
                            ROS_INFO("case3");
                            ROS_INFO("norm %f",norm);
            			  p_i=3;
                          //steering_command.data=270;//defelxión lado contrario
                          steering_command.data=180;//defelccion derecha
                           steering_publisher.publish(steering_command);  
                          steering_publisher.publish(steering_command);
            		    }
            			
            		break;
            		case 3:

            			if(bck1<=0.27&&bck1>0.1||bck2<=0.27&&bck2>0.1||bck3<=0.27&&bck3>0.1||norm>=0.82||msg->twist.twist.linear.y<(theta_ini*180/3.1416)){

                    
printf("angu2 %f,%f\n",msg->twist.twist.linear.y,norm );

                        //if(norm>=0.78||msg->twist.twist.linear.y<theta_ini*180/3.1416){
                            ROS_INFO("norm %f",norm);
                          ROS_INFO("llegue atras");
                          ROS_INFO("bck1 %f",bck1); 
                          ROS_INFO("bck2 %f",bck2);
                          ROS_INFO("bck3 %f",bck3);
            			  p_i=4;
            			  speed_command.data=0;//stop
                          speed_publisher.publish(speed_command);//detenemos

                          if(msg->twist.twist.linear.y>(theta_ini*180/3.1416)+2){
                            steering_command.data=60;//-70;//defelxión lado contrario
                            //speed_command.data=-580;//reversa
                            //speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                       }else{
                          steering_command.data=94;//-70;//defelxión lado contrario
                            //speed_command.data=-580;//reversa
                            //speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                       }
            			 
            		    }
            			
            		break;
            		case 4:    			
            			if (flag_p==false){
            				flag_p=true;
            				last_time = ros::Time::now();
                            ROS_INFO("case4");
                            ROS_INFO("norm %f",norm);
            			}
            			if((ros::Time::now()-last_time).toSec()>0.3){
            				//x_p=msg->pose.pose.position.x;
            				//y_p=msg->pose.pose.position.y;
            				p_i=5;
                            //steering_command.data=-30;//-70;//defelxión lado contrario
                            
                            speed_command.data=-480;//reversa
                            speed_publisher.publish(speed_command);//detenemos
                            
                            speed_publisher.publish(speed_command);//detenemos
                          
            			}
                        
            		break;
            		case 5:
            		   //px2=x_p-msg->pose.pose.position.x;
            	       //py2=y_p-msg->pose.pose.position.y;                        
            	       //norm=sqrt((px2*px2)+(py2*py2));
                    if(msg->twist.twist.linear.y>(theta_ini*180/3.1416)+3){
                            steering_command.data=60;//-70;//defelxión lado contrario
                            //speed_command.data=-580;//reversa
                            //speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                       }else{
                          steering_command.data=94;//-70;//defelxión lado contrario
                            //speed_command.data=-580;//reversa
                            //speed_publisher.publish(speed_command);//detenemos
                            steering_publisher.publish(steering_command);
                       }
            		if (parked==false){
            	       
                       ROS_INFO("norm %f",norm);
            			//if(nor0.012){//0.28
                       //theta_ini=-1*msg->twist.twist.linear.y*3.1416/180;
                       
                       if(fr1<=0.33&&fr1>0.1||fr2<=0.33&&fr2>0.1||fr3<=0.33&&fr3>0.1||norm<0.69){
            			  //p_i=4;
                        printf("angu2 %f,%f\n",msg->twist.twist.linear.y,norm );
                        ROS_INFO("llegué adelante");
                        ROS_INFO("fr1 %f",fr1);
                        ROS_INFO("fr2 %f",fr2);
                        ROS_INFO("fr3 %f",fr3);
                        

            			  parked=true;
            			  
            			  speed_command.data=0;//reversa    
                          steering_publisher.publish(steering_command);
                          speed_publisher.publish(speed_command);//detenemos
                          steering_publisher.publish(steering_command);
                          speed_publisher.publish(speed_command);//detenemos
                          
                          
            			  }		    
            		}else{
            			  steering_command.data=96;//defelxión lado contrario
            			  speed_command.data=0;//reversa

            		}

            		break;
            	    }
            
            	    c_d++;
            		if(c_d%40==0){
            		    ROS_INFO("n %f",norm);
            		    ROS_INFO("yawer2 %f,%f,%f",error_yaw,err_x,err_y);	    	    	
                    }

                }   
    	}
    }
void headingCallback(const std_msgs::Float32& msg)
{
  //yaw=msg.data;
}
	
	
 //    void headCallback(const std_msgs::Float32& head)
	// {
		
			
	// }
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    
        int count = scan->scan_time / scan->time_increment;//numero de datos 360
	    if (init_search_place==false){
            init_search_place=true;
        ROS_INFO("Scanner Init");
        speed_command.data=0;
        steering_command.data=96;//defelccion derecho
        speed_publisher.publish(speed_command);//detenemos
        steering_publisher.publish(steering_command);
        }
	    if (found_park==false){
                for(int i = 268; i < 272; i++){
                        if(scan->ranges[i] < 0.6 && scan->ranges[i]>.05){ //encontr un objeto proximo
                                if(obj2==true){
                                    norm2=sqrt((px-obj_x1)*(px-obj_x1)+(py-obj_y1)*(py-obj_y1));
                                    //ROS_INFO("n2 %.2f ",norm2);
                                    if(norm2>0.6){
                                        ROS_INFO("ESTACIONAMIENTO DISPOINBLE¡¡¡");
                                        spx=px+0.1*sin(theta_ini);
                                        spy=py+0.1*cos(theta_ini);
                                        speed_command.data=-200;//delante para buscar espacio de estacionamiento
                                        speed_publisher.publish(speed_command);//
                                        found_park=true;
                                    }
                                    obj2=false;
                                }
                                obj1=true;
                                speed_command.data=-700;//delante para buscar espacio de estacionamiento
                                        speed_publisher.publish(speed_command);//
                                //ROS_INFO("OBJ %.2f,%.2f ",px,py);
                        }else{
                            if(obj1==true){
                               obj_x1=px;
                               obj_y1=py; 
                               obj2=true;
                               obj1=false;  
                            }                             
                        }

                }

                
                

		} else{

            float sum=0;
            int index=0;
            int ka=0;
            int ang_1=0;
            float dist_prom=0.5;

            for(int i = 145; i > 120; i--){                    
                    if(scan->ranges[i] < 0.5 && scan->ranges[i] > .05 && index < 5){                        
                        sum = scan->ranges[i] + sum;
                        index = index + 1;
                        if(ka==0){
                            ang_1 = i;
                            ka=1;
                        }
                    }
            }

            dist_prom = sum / 5;
            if(ang_1 > 137 && ang_1 < 145 && norm>0.25 && dist_prom<.35 ){ 
                ROS_INFO("STOP¡ PARKING ROUTINE");
                flag_can_start=true;
            }
            int in=0;
    		for(int i = 0; i < (angle_front/2)+1; i++){
    				p_reg[in]=scan->ranges[i];
    			    in++;
    		}
    		for(int k = (360-(angle_front/2)); k < 360; k++){
    			    p_reg[in]=scan->ranges[k];
    			    in++;
    		}
    		fr1=scan->ranges[0];
    		fr2=scan->ranges[355];
    		fr3=scan->ranges[5];
    		bck1=scan->ranges[175];
    		bck2=scan->ranges[180];
    		bck3=scan->ranges[185];
    		
	
		}
	}
	
	
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    std_msgs::String light_command;
    std_msgs::Int16 steering_command;
    
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
    ros::Subscriber odometry_sub;
	
	ros::Time current_time, last_time;
    bool obj1,obj2;	

    float short_distance,min_d1,min_d2,cat_op_1,cat_op_2,parking_place,alpha,theta,ang_1,ang_2;

	float error_yaw,sp_yaw,ctrl_yaw;
	float px,py,norm,norm2,x_ini,y_ini,x_p,y_p,px2,py2;
	int direction,p_i,c_d,c_s;
	ros::Subscriber subScan_;
	ros::Subscriber subTwist_;
	float ctrl_speed;
	float fr1,fr2,fr3,bck1,bck2,bck3;
	int angle_front;
	int angle_back;
	float break_distance;
	bool init_yaw;
	bool obj_back;
	bool init_park,init_pf,flag_p,flag_p1,flag_p2,flag_p3,flag_p4,parked,flag_np,flag_can_start,flag_motor_has_started;
    bool place_found,init_search_place;
		
    float p_reg[61],num[61],temp;
	float xc,yc,prom_min;
    bool found_park;
    float theta_ini;
    float spx_ini,spy_ini, err_y,err_x,yaw,spx,spy,last_error;
    float obj_x1,obj_y1,obj_x2,obj_y2;
    float newangu;

		

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_speed_follow_node");
    ros::NodeHandle nh; 

    parking parking_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(40); //100 Hz, every .01 second
	//ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin();

	//ros::spin();
    std_msgs::Int16 steer_command;
    steer_command.data=96;//defelccion derecha
    
    parking_obj.steering_publisher.publish(steer_command);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();		
	}
    return 0;
}
