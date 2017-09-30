#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
class object_follow
{
public:
	object_follow(ros::NodeHandle nh)
	{
	    //obstacle_publisher=nh.advertise<std_msgs::String>(nh.resolveName("obstacle_front"),1);
		joy_subscriber = nh.subscribe("joy", 5, &object_follow::joyCallback,this);
		speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		steer_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 1);
		odometry_sub = nh.subscribe("odom",1,&object_follow::ReceiveOdometry,this);
		//angle_front=30;

	}
	~object_follow(){}

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
        std_msgs::Int16 speed;
		speed.data=-joy->axes[1]*1000;
		std_msgs::Int16 steer;
		steer.data=96+(joy->axes[2]*115);
		speed_publisher.publish(speed);
		steer_publisher.publish(steer);
	}


	void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg){
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;  
    	    
    	    count_odo++;
    	    if(count_odo%900==0){
    	    	printf("P%f,%f\n",px,py );
    	    }
    }
	
	
	
private:
    ros::Subscriber joy_subscriber;
    ros::Publisher speed_publisher;
    ros::Publisher steer_publisher;
    ros::Subscriber odometry_sub;
    int speed_rx;
    int count_odo;
    float px,py;
    
	
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gamepad_node");
    ros::NodeHandle nh; 

    object_follow object_follow_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	//ros::Rate rate(10);//100 Hz, every .01 second
	ROS_INFO("trabajando gamepad");	
	ros::spin();
	while(ros::ok())
	{
		//ros::spin();
		//ros::spinOnce();
		//rate.sleep();
	}
    return 0;
}
