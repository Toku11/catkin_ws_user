#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Joy.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h> 

class test_head
{
public:
	test_head(ros::NodeHandle nh)
	{
	    //light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        //speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
	    //visual_subscriber = nh.subscribe("visual_odometry", 1, &test_head::visualCallback,this);
		subTwist_ = nh.subscribe("odom",1,&test_head::speedCallback,this); 
		odometry_sub = nh.subscribe("visual_odometry",1,&test_head::ReceiveOdometry,this);
		//joy_subscriber = nh.subscribe("joy", 5, &test_head::joyCallback,this);
		//speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
		fp = fopen ("camara.txt", "w+");
		fd = fopen ("velocidad.txt", "w+");
		ROS_INFO("Listo para comenzar a sacar covariancia");	
		c1=0;
		c2=0;

	}
	~test_head(){}
	
	void speedCallback(const nav_msgs::Odometry::ConstPtr& twist)
	{
    	px=twist->pose.pose.position.x;
    	py=twist->pose.pose.position.y;  
    	yaw=twist->pose.pose.position.z;
		if(c1>=10000){
			ROS_INFO("Terminado de grabar velocidad");
		    fclose(fd);	
        }else{
           fprintf(fd,"%f,%f,%f\n",px,py,yaw);
           c1++;
        }
			
	}

    
	void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg){
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;  
    	    yaw=msg->pose.pose.orientation.z;
    	 	if(c2>=100000){
		    fclose(fp);	
		    ROS_INFO("Terminado de grabar odometria visual");
        	}else{
             fprintf(fp,"%f,%f,%f\n",px,py,yaw);
             
             c2++;
            }
    	    
    	   
    }

    

private:


	ros::Subscriber subTwist_;
	ros::Subscriber odometry_sub;
    int64_t c1,c2;
    FILE * fp;
    FILE * fd;
    float px,py,yaw;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_follow_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	//ros::Rate rate(50); //100 Hz, every .01 second
    ros::spin();
	//while(ros::ok())
	//{
		
	//	ros::spin();
		//rate.sleep();
	//}
    return 0;
}
