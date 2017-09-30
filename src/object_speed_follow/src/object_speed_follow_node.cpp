#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
class object_follow
{
public:
	object_follow(ros::NodeHandle nh)
	{
	    obstacle_publisher=nh.advertise<std_msgs::String>(nh.resolveName("obstacle_front"),1);
	    //steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 10);
        //speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
        steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("lane_steering_cmd"), 1);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("lane_speed_cmd"), 1);
		subScan_ = nh.subscribe("scan", 1, &object_follow::scanCallback,this);
		sub_cross= nh.subscribe("cross", 1, &object_follow::crossCallback,this);
	    //head_subscriber = nh.subscribe("model_car/yaw", 1, &object_follow::headCallback,this);
		subTwist_ = nh.subscribe("motor_control/twist",1,&object_follow::speedCallback,this); 
		//pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		angle_front=30;
		angle_back=20;
		break_distance=1.00;
		init_yaw=false;
		vel_crucero = 600;
		obj_izq=false,obj_der=false,obstacle_flag=false;
		vel_max=600;
		prom_min=5;	
	}
	~object_follow(){}

	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}

	void crossCallback(const std_msgs::Int16& data)
	{
		if(data.data==0){
			//no hay crucero
			cross_flag=false;
			vel_crucero=600;
		}else if (data.data==1){
			//hay crucero
			cross_flag=true;
			//vel_crucero=200;
		}
	}
	
 //    void headCallback(const std_msgs::Float32& head)
	// {
			
	// }
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;//numero de datos 360
	    
	    //if (abs(direction)>500)
	    //	break_distance=(abs(direction)/500)*break_distance;
	    std_msgs::Int16 speed;
	    speed.data=0;
	    int flag;
		
		int in=0;
		if(cross_flag==true){
			vel_crucero=200;
			
		    for(int i = 20; i < 41; i++){
		    	if(scan->ranges[i]>0.2&&scan->ranges[i]<0.8){		
			    in++;
			    }
			}	
			if(in>5){
			//obstacle_command.data="obstaculo";
			//vel_crucero=0;
			obj_der=true;
		}else{
			//vel_crucero=200;
			obj_der=false;
		}		
		in=0;
			for(int k = (320); k < 340; k++){
			    if(scan->ranges[k]>0.2&&scan->ranges[k]<0.8){		
			    in++;
			}
		}		
		if(in>5){
			//obstacle_command.data="obstaculo";
			//vel_crucero=0;
			obj_izq=true;
		}else{
			//vel_crucero=200;
			obj_izq=false;
		}
		
		}
	    	//ROS_INFO("speed %f",break_distance_);	
			///checamos region frontal por 
			int in1=0;
			for(int i = 0; i < (angle_front/2)+1; i++){
				p_reg[in]=scan->ranges[i];
			    in1++;
		}
		for(int k = (360-(angle_front/2)); k < 360; k++){
			    p_reg[in]=scan->ranges[k];
			    in1++;
		}
		
		//ordenamos para obtener los mínimos
		
		//int numElem = p_reg.length( );
		//int numElem = sizeof(p_reg)/sizeof(p_reg[0]);
		
        for(int i = 1; (i <= 61) && flag; i++){
          flag = 0;
          for (int k=0; k < (61 -1); k++){
               if (p_reg[k+1] < p_reg[k]){     // ascending order simply changes to <
                    temp = p_reg[k];             // swap elements
                    p_reg[k] = p_reg[k+1];
                    p_reg[k+1] = temp;
                    flag = 1;               // indicates that a swap occurred.
               }
          }
        }
		
	
		prom_min=(p_reg[0]+p_reg[1]+p_reg[2])/3;
		if(prom_min>5||prom_min<0.1){
			prom_min=5;
		}
        obstacle_flag=false;
		if(prom_min<=(break_distance)){
			obstacle_command.data="obstaculo";
			obstacle_publisher.publish(obstacle_command);
			obstacle_flag=true;
		}

	    if(obstacle_flag==true){
		ctrl_speed=1000*(break_distance-prom_min);
	    }else if(obj_der||obj_izq &&cross_flag){
	    	ctrl_speed=0;
	    }else if(cross_flag==true){
	    	ctrl_speed=-200;
	    }else{
	    	//ctrl_speed=-600;
	    	ctrl_speed=1000*(break_distance-prom_min);
	    }
				//pubEmergencyStop_.publish(speed);
				//ROS_INFO("Obstacle");
			if(ctrl_speed<-vel_max){
			   ctrl_speed=-vel_max;
			}else if(ctrl_speed>0){
			   ctrl_speed=0;
		}

		//if(direction > 0){ //detectamos objeto atrás
		//	for(int j = (180-(angle_back/2)); j < (180+(angle_back/2))+1; j++){
		//		if (scan->ranges[j] <= 0.4&&scan->ranges[j] >0){//detectamos objeto atrás
					
		//			speed.data=0;
		//			speed_publisher.publish(speed);
		//			return;
		//	    }
		//	}
		//}
		
		
		
		
		speed.data=ctrl_speed;
		speed_publisher.publish(speed);

		
		
		
	}
	
	
private:
    ros::Subscriber head_subscriber;
    ros::Publisher light_publisher;
    ros::Publisher obstacle_publisher;
    std_msgs::String light_command;
    ros::Publisher steering_publisher;
    std_msgs::Int16 steering_command;
    ros::Publisher speed_publisher;
    std_msgs::Int16 speed_command;
	std_msgs::String obstacle_command;
	ros::Subscriber sub_cross;
	
	
	float yaw,error_yaw,sp_yaw,ctrl_yaw;
	int direction;
	ros::Subscriber subScan_;
	ros::Subscriber subTwist_;
	float ctrl_speed;
	int angle_front;
	int angle_back;
	int vel_crucero;
	float break_distance;
	bool init_yaw;
	bool obj_back;
	float prom;
	bool cross_flag;
    float p_reg[61],num[61],temp,fr[20],fl[20];
	float xc,yc,prom_min;
	bool flag_fr,flag_fl;
	bool obj_izq,obj_der,obstacle_flag;
	int vel_max;

		

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_speed_follow_node");
    ros::NodeHandle nh; 

    object_follow object_follow_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(15); //100 Hz, every .01 second
	ROS_INFO("trabajando object_speed_follow_node");	
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
