#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
class master_speed_steering
{
public:

	ros::Subscriber lane_steering_subscriber;
    ros::Subscriber lane_speed_subscriber;
    ros::Subscriber parking_steering_subscriber;
    ros::Subscriber parking_speed_subscriber;
    
    std_msgs::String msg_command;
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
	std_msgs::String obstacle_command;
	
	bool speed_park_flg,speed_lane_flg,steer_park_flg,steer_lane_flg;

	master_speed_steering(ros::NodeHandle nh)
	{
	    
	    steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		lane_steering_subscriber = nh.subscribe("lane_steering_cmd", 2, &master_speed_steering::laneSteeringCallback,this);
		lane_speed_subscriber = nh.subscribe("lane_speed_cmd", 2, &master_speed_steering::laneSpeedCallback,this);
		parking_steering_subscriber = nh.subscribe("parking_steering_cmd", 2, &master_speed_steering::parkingSteeringCallback,this);
		parking_speed_subscriber = nh.subscribe("parking_speed_cmd", 2, &master_speed_steering::parkingSpeedCallback,this);
		speed_park_flg=false;
		speed_lane_flg=false;
		steer_lane_flg=false;
		steer_park_flg=false;
		steering_command.data=98;
	    
	}
	~master_speed_steering(){}

	void laneSpeedCallback(const std_msgs::Int16& data)
	{
			speed_lane_flg=true;
			if(speed_park_flg==false){
                 speed_command.data=(int)data.data;
		    }
			

	}
	
    void laneSteeringCallback(const std_msgs::Int16& data)
	{
		steer_lane_flg=true;	
		if(steer_park_flg==false){
           steering_command.data=(int)data.data;
		}
		
	}
	
	void parkingSpeedCallback(const std_msgs::Int16& data)
	{
		speed_park_flg=true;
		speed_command.data=(int)data.data;
		speed_publisher.publish(speed_command);//detenemos

	}
	
    void parkingSteeringCallback(const std_msgs::Int16& data)
	{
		steer_park_flg=true;
		steering_command.data=(int)data.data;
		steering_publisher.publish(steering_command);
		
	}
	

    
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "master_speed_steering_node");
    ros::NodeHandle nh;
    int c=0;

    master_speed_steering object(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(15); //100 Hz, every .01 second
	ROS_INFO("trabajando master_speed_steering_node");	
	int num_ele=4;
	while(ros::ok())
	{	

        //if(object.steer_park_flg==true){
		//	object.steering_publisher.publish(steering_command);
	    //}
		if (object.speed_park_flg!=true||object.steer_park_flg!=true){
		    object.steering_publisher.publish(object.steering_command);
            c++;
		  if(c%num_ele){
			object.speed_publisher.publish(object.speed_command);//detenemos
            
		 }
	   }
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
