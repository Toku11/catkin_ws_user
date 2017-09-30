#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//parameters
#include <ros/ros.h>

class auto_stop
{
public:
	auto_stop(ros::NodeHandle nh)
	{
		n_.param<int>("angle_right", angle_right, 160);
		n_.param<int>("angle_left", angle_left, 160);
		n_.param<float>("short_distance", short_distance, 2);//0.35
		pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		subScan_ = n_.subscribe("scan", 1, &auto_stop::scanCallback,this);
		subTwist_ = n_.subscribe("motor_control/twist",1,&auto_stop::speedCallback,this); 
	}
	~auto_stop(){}

    void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;
//        ROS_INFO("call¡");
	    std_msgs::Int16 speed;
	    speed.data = 0;
//	    ROS_INFO("count %i",count);	
	    min_d1=2;
	    min_d2=2;
	    ang_1=0;
	    ang_2=0;
	    cat_op_1=0;
	    cat_op_2=0;
	    parking_place=0;

for(int i = 20; i < 160; i++){
					
		 			if(scan->ranges[i] < min_d1 && scan->ranges[i]>.05 ){
		 				//if min_d1<min_d11 {
		 					dis = scan->ranges[i];
		 					ang_1 = i;
		 					ROS_INFO("v: %.2f a: %d",dis,i);
		 				//}
		 		}
		}


		// 	for(int i = 20; i < 90; i++){
					
		// 			if(scan->ranges[i] < min_d1 && scan->ranges[i]>.05 && scan->ranges[i]<.4){
		// 				//if min_d1<min_d11 {
		// 					min_d1 = scan->ranges[i];
		// 					ang_1 = i;
		// 				//}
		// 			}

		// 	}
		// 	for(int j = 90; j < 160; j++){
		// 			if(scan->ranges[j] < min_d2 && scan->ranges[j]>.05&& scan->ranges[j]<.4){

		// 				min_d2 = scan->ranges[j];
		// 				ang_2 = j;
		// 				}

		// 	}

		// if(ang_1 > 19){
				
		// 	//ROS_INFO("Min Dist 1: %.2f angle: %d",min_d1,ang_1);
		// 	theta = 90-ang_1;
		// 	theta = (theta*3.1416)/180;
		// 	cat_op_1 = sin(theta)*min_d1;
		// 	//ROS_INFO("Cateto 1: %.2f ",cat_op_1);	
		// }
			
		// if(ang_2 > 90){
			
		// 	//ROS_INFO("Min Dist 2: %.2f angle: %d",min_d2,ang_2);	
		// 	alpha = ang_2-90;
		// 	alpha = (alpha*3.1416)/180;
		// 	cat_op_2 = sin(alpha)*min_d2;
		// 	//ROS_INFO("Cateto 2: %.2f ",cat_op_2);	
		// }

		// parking_place = cat_op_1 + cat_op_2;
 	// 	ROS_INFO("P: %.2f ",parking_place);	 
		// if(parking_place > 0.62){ 
		// 	ROS_INFO("ESTACIONAMIENTO DISPOINBLE¡¡¡");}

	}

	private:
	  	int angle_right;
		int angle_left;
		float short_distance;
		float min_d1;
		float min_d2;
		float cat_op_1;
		float cat_op_2;
		float parking_place;
		float alpha;
		float theta;
		float ang_1;
		float ang_2;
		float dis;
		int direction;
	  	ros::NodeHandle n_; 
	  	ros::Publisher pubEmergencyStop_;
	  	ros::Subscriber subScan_;
	  	ros::Subscriber subTwist_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_place_node");
    ros::NodeHandle nh; 
    auto_stop autoStopObject(nh);
ros::Rate loop_rate(10);
//ROS_INFO("Hola cochesito");
	while(ros::ok())
	{
//ROS_INFO("Hola cochesito chingon!");

		ros::spinOnce();
loop_rate.sleep();
	}
    return 0;
}
