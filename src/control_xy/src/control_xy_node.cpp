#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
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
	    steering_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/steering"), 2);
        speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 2);
	    head_subscriber = nh.subscribe("model_car/yaw2", 1, &test_head::headCallback,this);
		subTwist_ = nh.subscribe("motor_control/twist",1,&test_head::speedCallback,this); 
		subAngle_ = nh.subscribe("angle_lane",1,&test_head::angleCallback,this);
		odometry_sub = nh.subscribe("odom",1,&test_head::ReceiveOdometry,this);
		joy_subscriber = nh.subscribe("joy", 3, &test_head::joyCallback,this);
		subScan_ = nh.subscribe("scan", 1, &test_head::scanCallback,this);
		speed_publisher=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 2);
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
		obstaculo=false;
		changed=false;
		correct=0;
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
	void angleCallback(const std_msgs::Float32& angle){
		if ((angle.data<8) && (angle.data>-8))
			correct=angle.data*.85;


	}

    void headCallback(const std_msgs::Float32& head)
	{	if(!loaded_route){
			printf("Ruta no cargada aun\n");
			loadRoute();
		}else if(ruta_learn==false&&loaded_route==true&&start_tray==true){
			std_msgs::Int16 speed;
			count2++;
			//calculamos error de heading
			float sumx=0,sumy=0,sumxy=0,sumx2=0,m=0,num=0,den=0,newy=0,norma2=0,signo=1,aux=0,aux2=0;
			int in;
			float n_aux2=30000;
			

			if(changed==true){
				time_obst=ros::Time::now();

				if(carril==0){//&& (time_obst-time_obst_last).toSec()>0.4&&changed==false) {
					//time_obst_last=time_obst;
				   carril=1;//cambiamos al interno
				   changed=false;
				   //changed=true;
				}else if(carril==1){//&& (time_obst-time_obst_last).toSec()>0.4,changed==false){
					//time_obst_last=time_obst;
					carril=0;
					changed=false;
					//changed=true;
				}//else if(changed=true){

				//}

			}

			if(crucero==true){
				if((ros::Time::now()-time_cruce_last).toSec()>7){
					crucero=false;
					//velocity=-600;
					k_idx=k_idx+1;
					speed_command.data=velocity;
					speed_publisher.publish(speed_command);
				}
			}

			if(carril==0){
				//determinar el mas cercano en 5 puntos
				int nat=0;
				float nerr_xx,nerr_yy;
				for (int at=k_idx;at<=k_idx+20;at++){
					nat=at;

					if(nat<=k_idx+5){
						if(param[nat]==0){
							//velocity=velocity/2;//prevenimos el stop;

						}	
					}

					if(at>=num_sp){
					nat=at-num_sp;
					}
					nerr_xx=sp_rx[nat]-px;
	        		nerr_yy=sp_ry[nat]-py;
					norma2=sqrt(nerr_xx*nerr_xx+nerr_yy*nerr_yy);
					if (norma2<n_aux2){
						n_aux2=norma2;
						k_idx=nat;
						if(param[nat]==0&&crucero==false){
							//velocity=0;//stop por crucero
							speed_command.data=0;
							speed_publisher.publish(speed_command);
							printf("det");
							time_cruce_last=ros::Time::now();
							crucero=true;
							k_idx=k_idx+1;
						}
						printf("act %d,%f,%f\n", k_idx, nerr_xx,nerr_yy);
						signo=-atan2(nerr_xx,nerr_yy);
						// aux=filter_yaw+50;
						// aux2=filter_yaw-50;
						// if (signo < -180.0){
	    	//     			signo=signo+360;
						// }else if (signo > 180.0 ){
		    //     			signo=signo-360;
						// }

						// if (aux < -180.0){
	    	//     			aux=aux+360;
						// }else if (aux > 180.0 ){
		    //     			aux=aux-360;
						// }

						// if (aux2 < -180.0){
	    	//     			aux2=aux2+360;
						// }else if (aux2 > 180.0 ){
		    //     			aux2=aux2-360;
						// }

						//if (signo<aux&&signo>aux2)
						//if (sp_yaww[nat]>=90&&sp_yaww[nat]<=-90)
						if (filter_yaw>=90||filter_yaw<=-90)
							signo=signo*-1;
						else
							signo=signo*1;
						if (signo<0) signo=-1;
						else signo=1;

	        			}	        		
	        	}
				spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];


		    } else{
//determinar el mas cercano en 5 puntos

				int nat=0;
				float nerr_xx,nerr_yy;
				for (int at=k_idx;at<=k_idx+20;at++){
					nat=at;
					
					if(at>=num_sp){
					nat=at-num_sp;
					}

					nerr_xx=sp_rx2[nat]-px;
	        		nerr_yy=sp_ry2[nat]-py;
					norma2=sqrt(nerr_xx*nerr_xx+nerr_yy*nerr_yy);
					if (norma2<n_aux2){
						n_aux2=norma2;
						k_idx=nat;

						//printf("act %d,%f,%f\n", k_idx, nerr_xx,nerr_yy);
						signo=-atan2(nerr_xx,nerr_yy);
						// aux=filter_yaw+50;
						// aux2=filter_yaw-50;
						// if (signo < -180.0){
	    	//     			signo=signo+360;
						// }else if (signo > 180.0 ){
		    //     			signo=signo-360;
						// }

						// if (aux < -180.0){
	    	//     			aux=aux+360;
						// }else if (aux > 180.0 ){
		    //     			aux=aux-360;
						// }

						// if (aux2 < -180.0){
	    	//     			aux2=aux2+360;
						// }else if (aux2 > 180.0 ){
		    //     			aux2=aux2-360;
						// }

						//if (signo<aux&&signo>aux2)
						//if (sp_yaww[nat]>=90&&sp_yaww[nat]<=-90)
						if (filter_yaw>=90||filter_yaw<=-90)
							signo=signo*-1;
						else
							signo=signo*1;
						if (signo<0) signo=-1;
						else signo=1;

	        			}	        		
	        	}
				spx=sp_rx[k_idx];
				spy=sp_ry[k_idx];
				
		    }
	        err_x=spx-px;
	        err_y=spy-py;

	        int net=k_idx+7;
			if(net>=num_sp){
					net=net-num_sp;
			}
	        sp_yaw=sp_yaww[net];

	        //sp_yaw2=-atan2(err_x,err_y)*180/3.1416;
	        

			yaw=filter_yaw;//head.data;//quitamos el offset
			//error_yaw=alfa*error_yaw+(1-alfa)*(sp_yaw-yaw);
			error_yaw=(sp_yaw-filter_yaw);
			float alfan=0.88;
			err2=err2*alfan+(1-alfan)*atan2(2.5*signo*n_aux2,velx)*180/3.14159;  //0.5*(sp_yaw2-filter_yaw);///(-1*velx);
	        if (err2>30)
	        err2=30;
			else if (err2<-30)
			err2=-30;
		
			if (error_yaw < -180.0 ){
	        	error_yaw=error_yaw+360;
			}else if (error_yaw > 180.0 ){
	        	error_yaw=error_yaw-360;
			}

			error_yaw=2.5*error_yaw+err2-correct;
			if (error_yaw < -180.0 ){
	        	error_yaw=error_yaw+360;
			}else if (error_yaw > 180.0 ){
	        	error_yaw=error_yaw-360;
			}
			
          
			//////////////////////////////
			if(error_yaw>70||error_yaw<-70){

			}else{
				direction=-70;//adelante
				speed_command.data=velocity;
			}
			//////////////////////


			if(direction<-30){//hacia delante
				ctrl_yaw=96+(1.0*(error_yaw));//+0.5*(error_yaw-last_error));//+0.5*(error_yaw-last_error));//2*(error_yaw);
			}
			else if (direction>30){//atrás
				
				ctrl_yaw=96-1*(error_yaw);//2*(error_yaw);
			}
			last_error=error_yaw;
			if(ctrl_yaw>180){
			    		ctrl_yaw=180; 
				 }else if(ctrl_yaw<1){
				 	ctrl_yaw=1; 
				 }
				 
			if(count2%4==0){
				//printf("%f,%f,%f,%f,%f\n",yaw,error_yaw,sp_yaw,signo*n_aux2,err2);
				//speed_command.data=velocity-10*abs(error_yaw);
				//speed_command.data=velocity;
				//speed_publisher.publish(speed_command);
				steering_command.data=(int)ctrl_yaw;
				steering_publisher.publish(steering_command);	
			}
		}	 
		
	}

	void ReceiveOdometry(const nav_msgs::Odometry::ConstPtr& msg){
    		px=msg->pose.pose.position.x;
    	    py=msg->pose.pose.position.y;  
    	    filter_yaw=msg->twist.twist.linear.y;
    	    velx=msg->twist.twist.linear.x;
    	    if (ruta_learn==true){
    	    	norm=sqrt((px-px_aux)*(px-px_aux)+(py-py_aux)*(py-py_aux));
    	    	if (norm>0.05){//0.87){
					float newAng=(filter_yaw+90);
					if (newAng < -180.0 ){
	        			newAng=newAng+360;
					}else if (newAng > 180.0 ){
	        			newAng=newAng-360;
					}
					newAng=newAng*3.1416/180;

    	    		fprintf(fp, "%f %f %f %f %f 2 \n",px,py,px-0.4*sin(newAng),py+0.4*cos(newAng),filter_yaw );//ambos carriles mas una bandera 2 para sin cambios
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





    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;
	    float  break_distance_=0.8;
	    
		if(obstaculo==false){
			for(int i = 0; i < (6/2)+1; i++){
				if (scan->ranges[i] <= 0.8&& scan->ranges[i]>.05){
					ROS_INFO("Obstacle");
					obstaculo=true;
					changed=true;
					return;
			    }
			}
			for(int k = (360-(6/2)); k < count; k++){
				if (scan->ranges[k] <= 0.8 && scan->ranges[k]>.05){
					obstaculo=true;
					//pubEmergencyStop_.publish(speed);
					ROS_INFO("Obstacle");
					changed=true;
					return;
			    }
			}
		}else{
			if (carril==1){
			for(int i = 275; i < 280; i++){
			if(scan->ranges[i] < 0.7 && scan->ranges[i]>.05){ //encontr un objeto proximo
						//obj1=true;
                                if(obj2==true){
                                	if(sqrt((px-obj_x1)*(px-obj_x1)+(py-obj_y1)*(py-obj_y1))>0.6){
                                			obstaculo=false;
                                			changed=true;
                                    		obj2=false;
                                    }
                                                                
                        }else{
                            //if(obj1==true){
                               obj_x1=px;
                               obj_y1=py; 
                               obj2=true;
                              // obj1=false;  
                            //}           

						}
					}
				}
		
		 }
		 else{
		 	obstaculo=false;
		 	changed=true;
		 }
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
	        			k_idx=num_sp-k_idx;
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
   		float newAng=(filter_yaw+90);
					if (newAng < -180.0 ){
	        			newAng=newAng+360;
					}else if (newAng > 180.0 ){
	        			newAng=newAng-360;
					}
					newAng=newAng*3.1416/180;

        if(joy->buttons[5]==1&&joy->buttons[7]==1){
					ruta_learn=false;
					ROS_INFO("Terminado de aprender la ruta");
					fclose(fp);	 
		}if(joy->buttons[3]==1){ //waypoint para incrementar velocidad
			fprintf(fp, "%f %f %f %f %f 3 \n",px,py,px-0.4*sin(newAng),py+0.4*cos(newAng),filter_yaw );//ambos carriles mas una bandera
    	    ROS_INFO("NP Aumenta velocidad %f,%f",px,py);	 
		}else if(joy->buttons[1]==1){ //waypoint para disminuir velocidad
			fprintf(fp, "%f %f %f %f %f 1 \n",px,py,px-0.4*sin(newAng),py+0.4*cos(newAng),filter_yaw );//ambos carriles mas una bandera
    	    ROS_INFO("NP Disminuye velocidad %f,%f",px,py);	 
		}else if(joy->buttons[0]){ //waypoint para stop
			fprintf(fp, "%f %f %f %f %f 0 \n",px,py,px-0.4*sin(newAng),py+0.4*cos(newAng),filter_yaw );//ambos carriles mas una bandera
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
		        	velocity=-800;
		        	speed.data=velocity;
		        	speed_publisher.publish(speed);
		        	
		        }else if(joy->buttons[7]==1){
					start_tray=false;
					velocity=0;
					speed.data=0;
		        	speed_publisher.publish(speed);
				}else if(joy->buttons[2]==1){
					carril=0;//externo
					k_idx+=2;//avanzamos uno extra en setpoint
				}else if(joy->buttons[0]==1){
					carril=1;//carril interno
					//obstaculo=true;
					k_idx+=2;//avanzamos uno extra en sp
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
	ros::Subscriber subScan_;
	ros::Subscriber subAngle_;
    int ca;
	int direction;
	float yaw;
	float error_yaw;
	float sp_yaw,sp_yaw2;
	float ctrl_yaw;
	float px,py,norm,norm2,x_ini,y_ini,x_p,y_p,px2,py2,spx,spy,err_x,err_y,err_xx,err_yy;
	char rx[80],ry[80],rx2[80],ry2[80],pa[80],yag[80];
	float sp_rx[10000],sp_ry[10000],sp_rx2[10000],sp_ry2[10000],param[10000],sp_yaww[10000];
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
	float velx,err2;
	ros::Time time_obst,time_cruce,time_obst_last,time_cruce_last;
	bool obstaculo,changed,crucero;
	float correct;
	float obj_y1,obj_x1;
	bool obj1,obj2;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heading_follow_node");
    ros::NodeHandle nh; 

    test_head test_head_obj(nh);
    //ros::Rate rate(0.5); //0.5 Hz, every 2 second
	ros::Rate rate(80); //100 Hz, every .01 second
    //printf("%f",-atan2(5,1)*180/3.1416);
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
