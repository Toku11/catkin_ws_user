#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
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
		//pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		angle_front=60;
		angle_back=30;
		break_distance=0.85;
		init_yaw=false;
	}
	~parking(){}

	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}
	
    void headCallback(const std_msgs::Float32& head)
	{
		
		if(init_yaw){
		
			yaw=head.data;
			error_yaw=(sp_yaw-yaw);
		
		
			if (error_yaw < -180.0 ){
        		error_yaw=error_yaw+360;
			}else if (error_yaw > 180.0 ){
	        	error_yaw=error_yaw-360;
			}
		
			if(direction<-2){//hacia delante
		
				ctrl_yaw=96-2*(error_yaw);
		
				if(ctrl_yaw>177){
		   			ctrl_yaw=178; 
				}else if(ctrl_yaw<3){
					ctrl_yaw=2; 
				}
		
			}
			else if (direction>2){//atrás
		
				ctrl_yaw=96+2*(error_yaw);
		
				if(ctrl_yaw>177){
		   			ctrl_yaw=178; 
				}else if(ctrl_yaw<3){
					ctrl_yaw=2; 
				}
			}
			steering_command.data=(int)ctrl_yaw;
			steering_publisher.publish(steering_command);
		
		}else{
			sp_yaw=head.data;
			init_yaw=true;
		}
		
		
		
		//ROS_INFO("error %f",error_yaw);	
		
		
		
		
//		if (head.data < 0.0 && head.data > -90.0){
//        		yaw=0;
//		}else if (head.data < 0.0 && head.data < -90.0){
//			yaw=180;
//		}else{
//			yaw=head.data;		
//		}     
//		steering_command.data=yaw;
//		steering_publisher.publish(steering_command);
		
//if(head.data<92&&head.data>88.0){
//speed_command.data=0;
//speed_publisher.publish(speed_command);//detenemos
//}else{
//speed_command.data=-300;
//speed_publisher.publish(speed_command);//avanzamos
//}		
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
		ROS_INFO("d1 %f",p_reg[0]);
		ROS_INFO("d2 %f",p_reg[1]);
		ROS_INFO("d3 %f",p_reg[3]);
		ROS_INFO("d4 %f",p_reg[59]);
		ROS_INFO("d1 %f",p_reg[58]);
		
			
		
		
//		ctrl_speed=1000*(break_distance-scan->ranges[i]);
//				//pubEmergencyStop_.publish(speed);
//				//ROS_INFO("Obstacle");
//			if(ctrl_speed<-700){
//			   ctrl_speed=-700;
//			}else if(ctrl_speed>500){
//			   ctrl_speed=500;
//		}
		
		
		
		
//		if(direction > 0){ //detectamos objeto atrás
//			for(int j = (180-(angle_back/2)); j < (180+(angle_back/2))+1; j++){
//				if (scan->ranges[j] <= 0.4){//detectamos objeto atrás
//					
//					speed.data=0;
//					speed_publisher.publish(speed);
//					return;
//			    }
//			}
//		}
		
		
		
		
//		speed.data=ctrl_speed;
//		speed_publisher.publish(speed);

		
		
		
	}
	
	
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    std_msgs::String light_command;
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
	
	
	
	float yaw,error_yaw,sp_yaw,ctrl_yaw;
	int direction;
	ros::Subscriber subScan_;
	ros::Subscriber subTwist_;
	float ctrl_speed;
	int angle_front;
	int angle_back;
	float break_distance;
	bool init_yaw;
	bool obj_back;
		
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
	while(ros::ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
