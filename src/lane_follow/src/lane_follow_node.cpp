#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
//#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
class object_follow
{
public:
	object_follow(ros::NodeHandle nh)
	{
	    //light_publisher=nh.advertise<std_msgs::String>(nh.resolveName("manual_control/lights"), 10);
	    //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        //speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
        steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("lane_steering_cmd"), 1);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("lane_speed_cmd"), 1);
		//subScan_ = nh.subscribe("scan", 1, &object_follow::scanCallback,this);
	    //head_subscriber = nh.subscribe("model_car/yaw", 1, &object_follow::headCallback,this);
		//subTwist_ = nh.subscribe("motor_control/twist",1,&object_follow::speedCallback,this); 
		subAngle_ = nh.subscribe("/angle_lane",1,&object_follow::angleCallback,this);
		//pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		angle_front=60;
		angle_back=30;
		break_distance=0.85;
		angleint=0;
		kp=0.7;
		kd=0.8;
		ki=0.00;
		alfa=0.75;
		//init_yaw=false;
	}
	~object_follow(){}

	void angleCallback(const std_msgs::Float32& angle)
	{
		angle_e=angle_e*alfa+(1-alfa)*angle.data;
		error_yaw=(angle_e);
			angleint=angleint+angle_e;
			if((angleint*ki)>5||(angleint*ki<-5)){
				angleint=0;
			}
			if(angle_e==0||(angle_e>0&&angle_e<1)||(angle_e<0&&angle_e>-1)){
				ctrl_yaw=98;
				}
			else{
				ctrl_yaw=98-((kp*angle_e)+(ki*angleint)+(kd*(angle_e-last_angle)));
				}
				last_angle=angle_e;
				
			steering_command.data=(int)ctrl_yaw;
			steering_publisher.publish(steering_command);
	}

	//void speedCallback(const geometry_msgs::Twist& twist)
	//{
//		direction=twist.linear.x;
//	}
	
 //    void headCallback(const std_msgs::Float32& head)
	// {
		

		
	// }
	
	// void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	// {
	//     int count = scan->scan_time / scan->time_increment;//numero de datos 360
	    
	//     //if (abs(direction)>500)
	//     //	break_distance=(abs(direction)/500)*break_distance;
	//     std_msgs::Int16 speed;
	//     speed.data=0;
		
	
		
	//     //ROS_INFO("speed %f",break_distance_);	
	// 	///checamos region frontal por 
	// 	int in=0;
	// 	for(int i = 0; i < (angle_front/2)+1; i++){
	// 			p_reg[in]=scan->ranges[i];
	// 		    in++;
	// 	}
	// 	for(int k = (360-(angle_front/2)); k < 360; k++){
	// 		    p_reg[in]=scan->ranges[k];
	// 		    in++;
	// 	}
		
	// 	//ordenamos para obtener los mínimos
		
	// 	//int numElem = p_reg.length( );
	// 	//int numElem = sizeof(p_reg)/sizeof(p_reg[0]);
	// 	int flag;
 //        for(int i = 1; (i <= 61) && flag; i++)
 //        {
 //          flag = 0;
 //          for (int k=0; k < (61 -1); k++)
 //          {
 //               if (p_reg[k+1] < p_reg[k])      // ascending order simply changes to <
 //               { 
 //                    temp = p_reg[k];             // swap elements
 //                    p_reg[k] = p_reg[k+1];
 //                    p_reg[k+1] = temp;
 //                    flag = 1;               // indicates that a swap occurred.
 //               }
 //          }
 //        }
	// 	//ROS_INFO("d1 %f",p_reg[0]);
	// 	//ROS_INFO("d2 %f",p_reg[1]);
	// 	//ROS_INFO("d3 %f",p_reg[3]);
	// 	//ROS_INFO("d4 %f",p_reg[59]);
	// 	//ROS_INFO("d1 %f",p_reg[58]);
		
			

		
		
		
	// }
	
	
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    std_msgs::String light_command;
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
    ros::Subscriber subAngle_;
	
	
	float alfa;
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
	float angle_e;
	float last_angle;
	float ki,kp,kd;
	float angleint;
		
    float p_reg[61],num[61],temp;
	float xc,yc,prom_min;

		

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_follow_node");
    ros::NodeHandle nh; 

    object_follow object_follow_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(100); //100 Hz, every .01 second
	while(ros::ok())
	{
		//rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
