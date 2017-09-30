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
	parking(ros::NodeHandle nh)
	{
	    //light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("parking_steering_cmd"), 1);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("parking_speed_cmd"), 1);
		subScan_ = nh.subscribe("scan", 1, &parking::scanCallback,this);
	    //head_subscriber = nh.subscribe("model_car/yaw", 1, &parking::headCallback,this);
		subTwist_ = nh.subscribe("motor_control/twist",1,&parking::speedCallback,this); 
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
        init_search_place=true;
	}
	~parking(){}

    void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg)
 	//void ReceiveOdometry(const nav_msgs::Odometry& msg)
    {
    	
    	//ROS_INFO("px %f",px);	
    	//ROS_INFO("py %f",py);
    	//ROS_INFO("Seq: [%d]", msg->header.seq);

    	
    	if (init_park==false&&found_park==true){
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;
    	    //norm=sqrt((px*px)+(py*py));
    		x_ini=px;
    		y_ini=py;
    		init_park=true;
            
            speed_command.data=-80;//reversa
            speed_publisher.publish(speed_command);//detenemos
            
            
            p_i=0;

    	}else if(found_park==true&&init_park==true){
    		px=x_ini-msg->pose.pose.position.x;
    	    py=y_ini-msg->pose.pose.position.y;
    	    norm=sqrt((px*px)+(py*py));
    	    

    	    switch(p_i){
            case 0:

                if (flag_np==false){    
                  if(norm>=0.46||flag_can_start==true){//nos desplazamos a la posicion de parking inicial
                   
                   speed_command.data=0;//stop
                   ROS_INFO("case0");   
                   steering_command.data=-40;//defelccion derecha
                   speed_publisher.publish(speed_command);//detenemos
                   steering_publisher.publish(steering_command);
                   speed_publisher.publish(speed_command);//detenemos
                   
                   steering_publisher.publish(steering_command);
                   last_time = ros::Time::now(); 
                   flag_np=true;           
                  }
              }else{
                  if((ros::Time::now()-last_time).toSec()>1.4){
                    x_ini=msg->pose.pose.position.x;                     
                    y_ini=msg->pose.pose.position.y;
                    p_i=1;                    
                  }
              }    
            break;    
    		case 1:
    		    if (flag_p1==false){
    				flag_p1=true;
    				
    				speed_command.data=60;//reversa
    			    //steering_command.data=-80;//defelccion derecha
                    steering_command.data=0;//defelccion derecha

                    speed_publisher.publish(speed_command);//detenemos
                    steering_publisher.publish(steering_command);
                    speed_publisher.publish(speed_command);//detenemos
                    steering_publisher.publish(steering_command);
                    
    			}

    			if(norm>=0.29){//0.28
    			  p_i=2;
    		    }


    		break;
    		case 2:
    		    if (flag_p2==false){
    				flag_p2=true;
    				last_time = ros::Time::now();    				
    			    steering_command.data=98;//defelccion centro
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
    			if(norm>=0.32){//0.28
    			  p_i=3;
                  //steering_command.data=270;//defelxión lado contrario
                  steering_command.data=220;//defelccion derecha
                 speed_command.data=60;//reversa 
                 speed_publisher.publish(speed_command);//detenemos
                   steering_publisher.publish(steering_command);  
                    speed_publisher.publish(speed_command);//detenemos
                  steering_publisher.publish(steering_command);
    		    }
    			
    		break;
    		case 3:

    			if(bck1<=0.27&&bck1>0.1||bck2<=0.27&&bck2>0.1||bck3<=0.27&&bck3>0.1||norm>=0.59){
                  ROS_INFO("llegue atras");
                  ROS_INFO("bck1 %f",bck1); 
                  ROS_INFO("bck2 %f",bck2);
                  ROS_INFO("bck3 %f",bck3);
    			  p_i=4;
    			  speed_command.data=0;//stop

                  speed_publisher.publish(speed_command);//detenemos
                   // steering_publisher.publish(steering_command);
                   // speed_publisher.publish(speed_command);//detenemos
                  //steering_publisher.publish(steering_command);
    			  //x_p=msg->pose.pose.position.x;
    			  //y_p=msg->pose.pose.position.y;
    			  
    		    }
    			
    		break;
    		case 4:    			
    			if (flag_p==false){
    				flag_p=true;
    				last_time = ros::Time::now();
    			}
    			if((ros::Time::now()-last_time).toSec()>1.4){
    				//x_p=msg->pose.pose.position.x;
    				//y_p=msg->pose.pose.position.y;
    				p_i=5;
                    //steering_command.data=-30;//-70;//defelxión lado contrario
                    steering_command.data=0;//-70;//defelxión lado contrario
                    speed_command.data=-60;//reversa
                    speed_publisher.publish(speed_command);//detenemos
                    steering_publisher.publish(steering_command);
                    speed_publisher.publish(speed_command);//detenemos
                  steering_publisher.publish(steering_command);
    			}
                
    		break;
    		case 5:
    		   //px2=x_p-msg->pose.pose.position.x;
    	       //py2=y_p-msg->pose.pose.position.y;                        
    	       //norm=sqrt((px2*px2)+(py2*py2));
    		if (parked==false){
    	       
               
    			//if(nor0.012){//0.28
               if(fr1<=0.33&&fr1>0.1||fr2<=0.33&&fr2>0.1||fr3<=0.33&&fr3>0.1||norm<0.49){
    			  //p_i=4;
                ROS_INFO("llegué adelante");
                ROS_INFO("fr1 %f",fr1);
                ROS_INFO("fr2 %f",fr2);
                ROS_INFO("fr3 %f",fr3);
                

    			  parked=true;
    			  steering_command.data=96;//defelxión lado contrario
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
    		if(c_d%5==0){
    		    ROS_INFO("n %f",norm);
    		    //ROS_INFO("px %f",px);	    	    	
            }
    	}
    }

	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
        if(abs(direction)<20){
            if(flag_motor_has_started==false){
               speed_command.data=-200;//reversa    
               speed_publisher.publish(speed_command);//detenemos
            }

        }else if(abs(direction)>40){
            flag_motor_has_started=true;
        }
	}
	
 //    void headCallback(const std_msgs::Float32& head)
	// {
		
			
	// }
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    
        int count = scan->scan_time / scan->time_increment;//numero de datos 360
	    if (init_search_place==true){
            //steering_command.data=96;//defelxión lado contrario
            //speed_command.data=-70;//reversa    
            //speed_publisher.publish(speed_command);//detenemos
            //steering_publisher.publish(steering_command);
            //speed_publisher.publish(speed_command);//detenemos
            //speed_publisher.publish(speed_command);//detenemos
            //speed_publisher.publish(speed_command);//detenemos
            //speed_publisher.publish(speed_command);//detenemos
            
            init_search_place=false;
        }
	    if (found_park==false){
            min_d1=1;
            min_d2=1;
            ang_1=0;
            ang_2=0;
            cat_op_1=0;
            cat_op_2=0;
            parking_place=0;
            
            


                for(int i = 20; i < 90; i++){
                        
                        if(scan->ranges[i] < min_d1 && scan->ranges[i]>.05 && scan->ranges[i]<.45){
                            //if min_d1<min_d11 {
                                min_d1 = scan->ranges[i];
                                ang_1 = i;
                            //}
                        }

                }
                for(int j = 90; j < 160; j++){
                        if(scan->ranges[j] < min_d2 && scan->ranges[j]>.05&& scan->ranges[j]<.45){

                            min_d2 = scan->ranges[j];
                            ang_2 = j;
                            }

                }

            if(ang_1 > 19){
                    
                //ROS_INFO("Min Dist 1: %.2f angle: %d",min_d1,ang_1);
                theta = 90-ang_1;
                theta = (theta*3.1416)/180;
                cat_op_1 = sin(theta)*min_d1;
                //ROS_INFO("Cateto 1: %.2f ",cat_op_1); 
            }
                
            if(ang_2 > 90){
                
                //ROS_INFO("Min Dist 2: %.2f angle: %d",min_d2,ang_2);  
                alpha = ang_2-90;
                alpha = (alpha*3.1416)/180;
                cat_op_2 = sin(alpha)*min_d2;
                //ROS_INFO("Cateto 2: %.2f ",cat_op_2); 
            }

            parking_place = cat_op_1 + cat_op_2;
            ROS_INFO("P: %.2f ",parking_place);  
            if(parking_place > 0.60){
                found_park=true; 
                ROS_INFO("ESTACIONAMIENTO DISPOINBLE¡¡¡");
            }


		} else{


            // for (int i =143; i>120;i--){

            //         if (scan->ranges[i]<.5 && scan->ranges[i]>0.05&& index1<5){
            //             sum1=scan->ranges[i]+sum1;
            //             index1=index1+1;
            //         }
                
            //     if (index1==5){
            //         break;
            //     }
                
            // }
            // prom1=sum1/index1;
            // if (prom1<0.35){
            //     ROS_INFO("  ALTO ¡¡¡ PUEDES EMPEZAR RUTINA DE ESTACIONAMIENTO");
            // }
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
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
    ros::Subscriber odometry_sub;
	
	ros::Time current_time, last_time;
	

    float short_distance,min_d1,min_d2,cat_op_1,cat_op_2,parking_place,alpha,theta,ang_1,ang_2;

	float yaw,error_yaw,sp_yaw,ctrl_yaw;
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

		

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_speed_follow_node");
    ros::NodeHandle nh; 

    parking parking_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(15); //100 Hz, every .01 second
	//ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin();

	//ros::spin();
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();		
	}
    return 0;
}
