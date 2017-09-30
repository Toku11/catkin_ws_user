#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class test_head
{
public:
	test_head(ros::NodeHandle nh)
	{
	    //light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
	    head_subscriber = nh.subscribe("model_car/yaw", 1, &test_head::headCallback,this);
		subTwist_ = nh.subscribe("motor_control/twist",1,&test_head::speedCallback,this); 
		sp_yaw=90;
	}
	~test_head(){}
	
	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}

    void headCallback(const std_msgs::Float32& head)
	{
		
		yaw=head.data;//quitamos el offset
		error_yaw=(sp_yaw-yaw);
		
		
		if (error_yaw < -180.0 ){
        	error_yaw=error_yaw+360;
		}else if (error_yaw > 180.0 ){
        	error_yaw=error_yaw-360;
		}
		
		if(direction<-30){//hacia delante
		
			ctrl_yaw=96-2*(error_yaw);
		
			if(ctrl_yaw>177){
		   		ctrl_yaw=178; 
			}else if(ctrl_yaw<3){
				ctrl_yaw=2; 
			}
		
		}
		else if (direction>30){//atrás
		
			ctrl_yaw=96+2*(error_yaw);
		
			if(ctrl_yaw>177){
		   		ctrl_yaw=178; 
			}else if(ctrl_yaw<3){
				ctrl_yaw=2; 
			}
		}
		steering_command.data=(int)ctrl_yaw;
		steering_publisher.publish(steering_command);
		
		
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
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    std_msgs::String light_command;
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
	ros::Subscriber subTwist_;
	int direction;
	float yaw;
	float error_yaw;
	float sp_yaw;
	float ctrl_yaw;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_follow_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(50); //100 Hz, every .01 second

	while(ros::ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
