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
	    head_subscriber = nh.subscribe("model_car/yaw2", 1, &test_head::headCallback,this);
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
		alfa=0.8;
	}
	~test_head(){}
	
	void speedCallback(const geometry_msgs::Twist& twist)
	{
		//direction=twist.linear.x;
		//std_msgs::Int16 speed;
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
			float sumx=0,sumy=0,sumxy=0,sumx2=0,m=0,num=0,den=0,newy=0;
			int in;


			if(carril==0){

			// for(int i=k_idx;i<=k_idx+2;i++){
			// 	in=i;
			// 	if(i>=num_sp){
			//    	   in=i-num_sp;
			//     }
			//     printf("%d,%f,%f,%f,%f,%f,%f\n",k_idx,sp_rx[in],sp_ry[in],sp_rx[in]-px,sp_ry[in]-py,px,py );
			// 	sumx+=(sp_rx[in]-px);
			// 	sumy+=(sp_ry[in]-py);
			// 	sumxy+=(sp_rx[in]-px)*(sp_ry[in]-py);
			// 	sumx2+=(sp_rx[in]-px)*(sp_rx[in]-px);
			// }
			//sumx+=px; //asi llevo a cero entonces seria sumar un cero pero serian 4 elementos
			//sumy+=py;
			//sumxy+=px*py;
			//sumx2+=px*px;
			//m=atan((4*sumxy-sumx*sumy)/(4*sumx2-sumx*sumx))*180/3.1416;
			// num=4*sumxy-sumx*sumy;
			// den=4*sumx2-sumx*sumx;
			// newy=(num/den)*sp_rx[k_idx+2];
			// m=-atan2(sp_rx[k_idx+2],newy)*180/3.1416;
			// printf("M%f,%f,%f\n",m,num,den);

				spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];


				for(int i=k_idx;i<=k_idx+3;i++){
					in=i;
					if(i>=num_sp){
			   	   		in=i-num_sp;
			    	}
			    	err_x=sp_rx[in]-px;
			    	err_y=sp_ry[in]-py;
			    	if(in==k_idx){
			    		m+=0.60*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+1){
			    		m+=0.21*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+2){
			    		m+=0.14*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+3){
			    		m+=0.05*(-atan2(err_x,err_y)*180/3.1416);
			    	}
			    }
			    //m=m/4;

		    } else{
		    	spx=sp_rx2[k_idx];
				spy=sp_ry2[k_idx];

				for(int i=k_idx;i<=k_idx+3;i++){
					in=i;
					if(i>=num_sp){
			   	   		in=i-num_sp;
			    	}
			    	err_x=sp_rx2[in]-px;
			    	err_y=sp_ry2[in]-py;
			    	if(in==k_idx){
			    		m+=0.60*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+1){
			    		m+=0.21*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+2){
			    		m+=0.14*(-atan2(err_x,err_y)*180/3.1416);
			    	}else if(in==k_idx+3){
			    		m+=0.05*(-atan2(err_x,err_y)*180/3.1416);
			    	}
			    }
		    }
	        err_x=spx-px;
	        err_y=spy-py;

	        if(err_x<.09 && err_x>-.09 && err_y<.09 && err_y>-.09){
			   
			   k_idx+=1; 
			   if(k_idx>=num_sp){
			   	k_idx=1;
			   }
			   printf("sig sp %d,%f,%f,%f \n",k_idx,err_x,err_y,sp_yaw);
			    spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];
	        	err_x=spx-px;
	        	err_y=spy-py;

	        	if(param[k_idx]==3){
	        		velocity-=50;
	        	}else if(param[k_idx]==1){
	        		velocity+=50;
	        	}else if(param[k_idx]==0){
	        		velocity=0;
	        	}
	        	//ROS_INFO("Next SP %d, sp%f,%f, p%f,%f, e%f,%f", k_idx,spx,spy,px,py,err_x,err_y);	
			}
	        //sp_yaw=atan2(err_y,err_x)*180/3.1416;
	        sp_yaw=m;
	        //sp_yaw=-atan2(err_x,err_y)*180/3.1416;
	        

			yaw=filter_yaw;//head.data;//quitamos el offset
			//error_yaw=alfa*error_yaw+(1-alfa)*(sp_yaw-yaw);
			error_yaw=sp_yaw-filter_yaw;
	        
				
			//ROS_INFO("%f",error_yaw);

			if (error_yaw < -180.0 ){
	        	error_yaw=error_yaw+360;
			}else if (error_yaw > 180.0 ){
	        	error_yaw=error_yaw-360;
			}
			
          
			//////////////////////////////
			if(error_yaw>70||error_yaw<-70){
				//direction=70;//atras
				//speed_command.data=200;
				k_idx+=2;
				printf("skip\n");
				//error_yaw+=180;//agregamos offset

				if (error_yaw < -180.0){
	    	    	error_yaw=error_yaw+360;
				}else if (error_yaw > 180.0 ){
		        	error_yaw=error_yaw-360;
				}


			}else{
				direction=-70;//adelante
				speed_command.data=velocity;
			}
			//////////////////////


			if(direction<-30){//hacia delante
				ctrl_yaw=96+(1.5*(error_yaw)+0.5*(error_yaw-last_error));//+0.5*(error_yaw-last_error));//2*(error_yaw);
			}
			else if (direction>30){//atrás
				
				ctrl_yaw=96-2*(error_yaw);//2*(error_yaw);
			}
			last_error=error_yaw;
			if(ctrl_yaw>180){
			    		ctrl_yaw=180; 
				 }else if(ctrl_yaw<1){
				 	ctrl_yaw=1; 
				 }
				 
			if(count2%2==0){
				//printf("%f,%f\n",sp_yaw,yaw);
				//ROS_INFO("%f,%f",px,py);	 
			//	ROS_INFO("%f,%f",error_yaw,sp_yaw);	 
				speed_publisher.publish(speed_command);
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
    	    	if (norm>0.10){//0.87){
    	    		fprintf(fp, "%f %f %f %f %f 2 \n",px,py,px*0.8,py*0.8,filter_yaw );//ambos carriles mas una bandera 2 para sin cambios
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
        			
        			sscanf(mystring,"%s %s %s %s %s %s",&rx,&ry,&rx2,&ry2,&yag,&pa);
        			sp_rx[h]=atof(rx);
        			sp_ry[h]=atof(ry);
        			sp_rx2[h]=atof(rx2);
        			sp_ry2[h]=atof(ry2);
        			sp_yaww[h]=atof(yag);
        			param[h]= atof(pa);
        			
        			//printf("%3d: %s", h, mystring);
        			printf("R %f,%f,%f \n",sp_rx[h],sp_ry[h],param[h]);
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
	        		k_idx=at+2;
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
   if(ruta_learn==true){
        if(joy->buttons[5]==1&&joy->buttons[7]==1){
					ruta_learn=false;
					ROS_INFO("Terminado de aprender la ruta");
					fclose(fp);	 
		}if(joy->buttons[3]==1){ //waypoint para incrementar velocidad
			fprintf(fp, "%f %f %f %f %f 3 \n",px,py,px*0.8,py*0.8,filter_yaw );//ambos carriles mas una bandera
    	    ROS_INFO("NP Aumenta velocidad %f,%f",px,py);	 
		}else if(joy->buttons[1]==1){ //waypoint para disminuir velocidad
			fprintf(fp, "%f %f %f %f %f 1 \n",px,py,px*0.8,py*0.8,filter_yaw );//ambos carriles mas una bandera
    	    ROS_INFO("NP Disminuye velocidad %f,%f",px,py);	 
		}else if(joy->buttons[0]){ //waypoint para stop
			fprintf(fp, "%f %f %f %f %f 0 \n",px,py,px*0.8,py*0.8,filter_yaw );//ambos carriles mas una bandera
    	    ROS_INFO("NP Stop velocidad %f,%f",px,py);	 
		}
		speed.data=-joy->axes[1]*1100;
		std_msgs::Int16 steer;
		steer.data=96+(joy->axes[0]*110);
		speed_publisher.publish(speed);
		steering_publisher.publish(steer);
	}else{


				if(joy->buttons[5]==1&&joy->buttons[7]==1 && ruta_learn==false){
					fp = fopen ("ruta.txt", "w+");
					ruta_learn=true;
					ROS_INFO("Listo para comenzar a aprender la ruta");	
					px_aux=1000000000000;
				    py_aux=1000000000000; 
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
					k_idx+=3;//avanzamos uno extra en setpoint
				}else if(joy->buttons[0]==1){
					carril=1;//carril interno
					k_idx+=3;//avanzamos uno extra en sp
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
	char rx[80],ry[80],rx2[80],ry2[80],pa[80],yag[80];
	float sp_rx[1000],sp_ry[1000],sp_rx2[1000],sp_ry2[1000],param[1000],sp_yaww[1000];
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
	float last_error,alfa;

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
