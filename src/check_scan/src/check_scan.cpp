#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//parameters
#include <ros/ros.h>

bool stopped_scan;
bool check_health;
float last_time_scan,current_time_scan;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	    //int count = scan->scan_time / scan->time_increment;
//last_time_scan=current_time_scan;
    //current_time_scan = ros::Time::now();
last_time_scan = ros::Time::now().toSec();
check_health=true;
}

void check_scan(){

current_time_scan=ros::Time::now().toSec();
if((current_time_scan-last_time_scan)>0.3&&check_health==true){
	ROS_INFO("stopped_scan");
	stopped_scan=true;
	system("rosnode kill /rplidarNode");
	system("roslaunch rplidar_ros rplidar.launch");
	check_health=false;
	//last_time_scan = ros::Time::now().toSec();
}else if(check_health==false){
	if((current_time_scan-last_time_scan)>1.5){

		ROS_INFO("stopped_scan");
		stopped_scan=true;
		system("rosnode kill /rplidarNode");
		system("roslaunch rplidar_ros rplidar.launch");
		check_health=false;
		last_time_scan = ros::Time::now().toSec();
	}
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_scan_node");
    ros::NodeHandle nh; 
    ros::Subscriber subScan_;
    subScan_ = nh.subscribe("scan", 1, scanCallback);
//    auto_stop autoStopObject(nh);
	ros::Rate loop_rate(5);
	stopped_scan=false;
	check_health=false;
	while(ros::ok())
	{
check_scan();
	ros::spinOnce();
loop_rate.sleep();
	}
    return 0;
}
