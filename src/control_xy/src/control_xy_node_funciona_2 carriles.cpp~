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
float filter_yaw=0;
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
		odometry_sub = nh.subscribe("odom",1,&test_head::ReceiveOdometry,this);
		joy_subscriber = nh.subscribe("joy", 5, &test_head::joyCallback,this);
		speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 10);
		start_tray=false;
		sp_yaw=90;
		ca=0;
		ruta_learn=false;
		loaded_route=false;
		n_aux=3000;
		k_idx=0;
		px_aux=1000000000000;
		py_aux=1000000000000;
		carril=0;//0 externo 1 interno
		velocity=0;

	}
	~test_head(){}
	
	void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
		std_msgs::Int16 speed;
		// count++;
		// if(count%5==0){
		// 	if (start_tray==true){
		// 		speed.data=-740;
		// 		speed_publisher.publish(speed);
		// 	}else{
		// 		speed.data=0;
		// 		speed_publisher.publish(speed);
		// 	}
	 //    }
	}

    void headCallback(const std_msgs::Float32& head)
	{	if(!loaded_route){
			printf("Ruta no cargada aun\n");
			loadRoute();
		}else if(ruta_learn==false&&loaded_route==true&&start_tray==true){


			std_msgs::Int16 speed;
			count2++;
			//calculamos error de heading
			if(carril==0){
				spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];
		    } else{
		    	spx=sp_rx2[k_idx];
				spy=sp_ry2[k_idx];
		    }
	        err_x=spx-px;
	        err_y=spy-py;

	        if(err_x<.15 && err_x>-.15 && err_y<.15 && err_y>-.15){
			   
			   k_idx++; 
			   if(k_idx>=num_sp){
			   	k_idx=1;
			   }
			    spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];
	        	err_x=spx-px;
	        	err_y=spy-py;
	        	//ROS_INFO("Next SP %d, sp%f,%f, p%f,%f, e%f,%f", k_idx,spx,spy,px,py,err_x,err_y);	
			}
	        //sp_yaw=atan2(err_y,err_x)*180/3.1416;
	        sp_yaw=-atan2(err_x,err_y)*180/3.1416;
			yaw=filter_yaw;//head.data;//quitamos el offset
			error_yaw=(sp_yaw-yaw);
	        
				
			//ROS_INFO("%f",error_yaw);

			if (error_yaw < -180.0 ){
	        	error_yaw=error_yaw+360;
			}else if (error_yaw > 180.0 ){
	        	error_yaw=error_yaw-360;
			}
			
			if(direction<-30){//hacia delante
				ctrl_yaw=96+2*(error_yaw);//2*(error_yaw);
			}
			else if (direction>30){//atrás
				
				ctrl_yaw=96-2*(error_yaw);//2*(error_yaw);
			}

			if(ctrl_yaw>180){
			    		ctrl_yaw=180; 
				 }else if(ctrl_yaw<1){
				 	ctrl_yaw=1; 
				 }
			if(count2%4==0){
				//ROS_INFO("%f,%f",px,py);	 
			//	ROS_INFO("%f,%f",error_yaw,sp_yaw);	 
				steering_command.data=(int)ctrl_yaw;
				steering_publisher.publish(steering_command);	
			}
		}	 
		
	}

	void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg){
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;  
    	    filter_yaw=msg->twist.twist.linear.y;
    	    if (ruta_learn==true){
    	    	norm=sqrt((px-px_aux)*(px-px_aux)+(py-py_aux)*(py-py_aux));
    	    	if (norm>0.85){//0.87){
    	    		fprintf(fp, "%f %f %f %f \n",px,py,px*0.8,py*0.8 );//ambos carriles mas una bandera
    	    		px_aux=px;
    	    		py_aux=py;
    	    		ROS_INFO("NP %f,%f",px,py);	 
    	    	}
    	    }
    	    count_odo++;
    	    if(count_odo%300==0){
    	    	printf("P%f,%f\n",px,py );
    	    }
    }

    void loadRoute(){
    	    char mystring [100];
			//ROS_INFO("Cargando Ruta");
			fp = fopen ("ruta.txt" , "r");
			int h=0;
			if (fp == NULL){ perror ("Error opening file");}
			h=0;
			while (1) {
       			 if (fgets(mystring,100, fp) == NULL){ 
       			 	break;
       			 }
        			
        			sscanf(mystring,"%s %s %s %s",&rx,&ry,&rx2,&ry2);
        			sp_rx[h]=atof(rx);
        			sp_ry[h]=atof(ry);
        			sp_rx2[h]=atof(rx2);
        			sp_ry2[h]=atof(ry2);
        			
        			//printf("%3d: %s", h, mystring);
        			printf("R %f,%f \n",sp_rx[h],sp_ry[h]);
        			h++;
			}
			printf("Ruta cargada %d elementos\n ",h);
			
			num_sp=h;
			//determinar el mas cercano
			n_aux=30000;
			for (int at=0;at<num_sp;at++){
				err_xx=sp_rx[at]-px;
	        	err_yy=sp_ry[at]-py;
	        	
	        	if(sqrt(err_xx*err_xx+err_yy*err_yy)<n_aux){
	        		n_aux=sqrt(err_xx*err_xx+err_yy*err_yy);
	        		k_idx=at+1;
	        		if(k_idx>=num_sp){
	        			k_idx=0;
	        		}	        		
	        	}

			}
			printf("Siguiente SP %d, p%f,%f, sp%f,%f\n", k_idx,px,py,sp_rx[k_idx],sp_ry[k_idx] );
			loaded_route=true;

    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std_msgs::Int16 speed;

		if(joy->buttons[5]==1&&joy->buttons[7]==1 && ruta_learn==false){
			fp = fopen ("ruta.txt", "w+");
			ruta_learn=true;
			ROS_INFO("Listo para comenzar a aprender la ruta");	
			px_aux=1000000000000;
		    py_aux=1000000000000; 
		}else if(joy->buttons[5]==1&&joy->buttons[7]==1 && ruta_learn==true){
			ruta_learn=false;
			ROS_INFO("Terminado de aprender la ruta");
			fclose(fp);	 
		}else if(joy->buttons[4]==1&&joy->buttons[5]==1 ){
			printf("Cargando Ruta \n");
			loadRoute();

		}else if(joy->buttons[6]==1){
        	start_tray=true;
        	velocity=-500;
        	speed.data=velocity;
        	speed_publisher.publish(speed);
        	
        }else if(joy->buttons[7]==1){
			start_tray=false;
			velocity=0;
			speed.data=0;
        	speed_publisher.publish(speed);
		}else if(joy->buttons[2]==1){
			carril=0;//externo
			k_idx+=1;//avanzamos uno extra en setpoint
		}else if(joy->buttons[0]==1){
			carril=1;//carril interno
			k_idx+=1;//avanzamos uno extra en sp
		}else if(joy->buttons[1]==1){
			velocity+=100;
			ROS_INFO("vel %d", velocity);
			speed.data=velocity;
        	speed_publisher.publish(speed);
		}else if(joy->buttons[3]==1){

			velocity-=100;
			ROS_INFO("vel %d", velocity);	
			speed.data=velocity;
        	speed_publisher.publish(speed);
		}


		if(ruta_learn==true){

			std_msgs::Int16 speed;
		speed.data=-joy->axes[1]*1100;
		std_msgs::Int16 steer;
		steer.data=96+(joy->axes[0]*110);
		speed_publisher.publish(speed);
		steering_publisher.publish(steer);
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
	ros::Subscriber subTwist_;
	ros::Subscriber odometry_sub;
	ros::Subscriber joy_subscriber;
    int ca;
	int direction;
	float yaw;
	float error_yaw;
	float sp_yaw;
	float ctrl_yaw;
	float px,py,norm,norm2,x_ini,y_ini,x_p,y_p,px2,py2,spx,spy,err_x,err_y,err_xx,err_yy;
	char rx[80],ry[80],rx2[80],ry2[80];
	float sp_rx[1000],sp_ry[1000],sp_rx2[1000],sp_ry2[1000];
	bool start_tray;
	int count,count2;
	bool loaded_route;
	int count_odo;
	FILE * fp;
	bool ruta_learn;
	float px_aux,py_aux;
	int num_sp;
	int k_idx;
	float n_aux;
	int carril;
	int velocity;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_follow_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(50); //100 Hz, every .01 second
    //printf("%f",-atan2(5,1)*180/3.1416);
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
