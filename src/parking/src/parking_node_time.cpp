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
	    steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
		subScan_ = nh.subscribe("scan", 1, &parking::scanCallback,this);
	    head_subscriber = nh.subscribe("model_car/yaw", 1, &parking::headCallback,this);
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
		parked=false;
		p_i=0;
		c_d=0;
	}
	~parking(){}

    void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg)
 	//void ReceiveOdometry(const nav_msgs::Odometry& msg)
    {
    	
    	//ROS_INFO("px %f",px);	
    	//ROS_INFO("py %f",py);
    	//ROS_INFO("Seq: [%d]", msg->header.seq);
    	
    	if (init_park==false){
    		//px=msg->pose.pose.position.x;
    	    //py=msg->pose.pose.position.y;
    	    //norm=sqrt((px*px)+(py*py));
    		//x_ini=px;
    		//y_ini=py;
    		init_park=true;
    	}else{
    		//px=x_ini-msg->pose.pose.position.x;
    	    //py=y_ini-msg->pose.pose.position.y;
    	    //norm=sqrt((px*px)+(py*py));
    	    //ROS_INFO("px %f",px);	
    	    //ROS_INFO("py %f",py);
    	    //ROS_INFO("n %f",norm);	
    	    switch(p_i){
    		case 0:
    		    if (flag_p1==false){
    				flag_p1=true;
    				last_time = ros::Time::now();
    				speed_command.data=200;//reversa
    			    steering_command.data=-80;//defelccion derecha
    			}
    			if((ros::Time::now()-last_time).toSec()>3.4){
    				//x_p=msg->pose.pose.position.x;
    				//y_p=msg->pose.pose.position.y;
    				p_i=1;
    			}
    		    
    			//if(norm>=0.1){//0.28

    			//if(norm>=0.1){//0.28
    			//  p_i=1;
    		    //}


    		break;
    		case 1:
    		    if (flag_p2==false){
    				flag_p2=true;
    				last_time = ros::Time::now();    				
    			    steering_command.data=96;//defelccion centro
    			}
    			if((ros::Time::now()-last_time).toSec()>1.4){
    				//x_p=msg->pose.pose.position.x;
    				//y_p=msg->pose.pose.position.y;
    				p_i=2;
    			}
    			//steering_command.data=96;//defelccion derecha
    			//if(norm>=0.15){//0.28
    			//if(norm>=0.17){//0.28
    			//  p_i=2;
    		    //}
    			
    		break;
    		case 2:
    			steering_command.data=270;//defelxión lado contrario
    			//if(norm>=0.24){//0.28    				
    			//if(norm>=0.27){//0.28 
    			 speed_command.data=140;//reversa   
    			if(bck1<=0.27||bck2<=0.27||bck3<=0.27){
    			  p_i=3;
    			  speed_command.data=0;//stop
    			  //x_p=msg->pose.pose.position.x;
    			  //y_p=msg->pose.pose.position.y;
    			  
    		    }
    			
    		break;
    		case 3:    			
    			if (flag_p==false){
    				flag_p=true;
    				last_time = ros::Time::now();
    			}
    			if((ros::Time::now()-last_time).toSec()>1.4){
    				//x_p=msg->pose.pose.position.x;
    				//y_p=msg->pose.pose.position.y;
    				p_i=4;
    			}
                
    		break;
    		case 4:
    		   //px2=x_p-msg->pose.pose.position.x;
    	       //py2=y_p-msg->pose.pose.position.y;                        
    	       //norm=sqrt((px2*px2)+(py2*py2));
    		if (parked==false){
    	       steering_command.data=-70;//defelxión lado contrario
               speed_command.data=-120;//reversa
    			//if(nor0.012){//0.28
               if(fr1<=0.3||fr2<=0.3||fr3<=0.3){
    			  //p_i=4;
    			  parked=true;
    			  steering_command.data=96;//defelxión lado contrario
    			  speed_command.data=0;//reversa    
    			  }		    
    		}else{
    			  steering_command.data=96;//defelxión lado contrario
    			  speed_command.data=0;//reversa
    		}

    		break;
    	    }

    	    //c_d++;
    		//if(c_d%2==0){
    		    //ROS_INFO("n %f",norm);
    	    	speed_publisher.publish(speed_command);//detenemos
    	    	steering_publisher.publish(steering_command);
            //}
    	}
    }

	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}
	
    void headCallback(const std_msgs::Float32& head)
	{
		
		// if(init_yaw){
		
		// 	yaw=head.data;
		// 	error_yaw=(sp_yaw-yaw);
		
		
		// 	if (error_yaw < -180.0 ){
  //       		error_yaw=error_yaw+360;
		// 	}else if (error_yaw > 180.0 ){
	 //        	error_yaw=error_yaw-360;
		// 	}
		
		// 	if(direction<-2){//hacia delante
		
		// 		ctrl_yaw=96-2*(error_yaw);
		
		// 		if(ctrl_yaw>177){
		//    			ctrl_yaw=178; 
		// 		}else if(ctrl_yaw<3){
		// 			ctrl_yaw=2; 
		// 		}
		
		// 	}
		// 	else if (direction>2){//atrás
		
		// 		ctrl_yaw=96+2*(error_yaw);
		
		// 		if(ctrl_yaw>177){
		//    			ctrl_yaw=178; 
		// 		}else if(ctrl_yaw<3){
		// 			ctrl_yaw=2; 
		// 		}
		// 	}
		// 	steering_command.data=(int)ctrl_yaw;
		// 	steering_publisher.publish(steering_command);
		
		// }else{
		// 	sp_yaw=head.data;
		// 	init_yaw=true;
		// }
			
	}
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;//numero de datos 360
	    
	    //if (abs(direction)>500)
	    //	break_distance=(abs(direction)/500)*break_distance;
	    std_msgs::Int16 speed;
	    speed.data=0;
		
	
		
	    //ROS_INFO("speed %f",break_distance_);	
		///checamos region frontal por 
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
		fr2=scan->ranges[358];
		fr3=scan->ranges[2];
		bck1=scan->ranges[178];
		bck2=scan->ranges[180];
		bck3=scan->ranges[182];
		//ROS_INFO("fr %f",scan->ranges[1]);	
		//ROS_INFO("bck %f",scan->ranges[180]);	
		//ordenamos para obtener los mínimos
		
		//int numElem = p_reg.length( );
		//int numElem = sizeof(p_reg)/sizeof(p_reg[0]);
		int flag;
        for(int i = 1; (i <= 61) && flag; i++)
        {
          flag = 0;
          for (int k=0; k < (61 -1); k++)
          {
               if (p_reg[k+1] < p_reg[k])      // ascending order simply changes to <
               { 
                    temp = p_reg[k];             // swap elements
                    p_reg[k] = p_reg[k+1];
                    p_reg[k+1] = temp;
                    flag = 1;               // indicates that a swap occurred.
               }
          }
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
	
	float yaw,error_yaw,sp_yaw,ctrl_yaw;
	float px,py,norm,norm2,x_ini,y_ini,x_p,y_p,px2,py2;
	int direction,p_i,c_d;
	ros::Subscriber subScan_;
	ros::Subscriber subTwist_;
	float ctrl_speed;
	float fr1,fr2,fr3,bck1,bck2,bck3;
	int angle_front;
	int angle_back;
	float break_distance;
	bool init_yaw;
	bool obj_back;
	bool init_park,init_pf,flag_p,flag_p1,flag_p2,flag_p3,flag_p4,parked;
		
    float p_reg[61],num[61],temp;
	float xc,yc,prom_min;

		

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_speed_follow_node");
    ros::NodeHandle nh; 

    parking parking_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(100); //100 Hz, every .01 second
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
