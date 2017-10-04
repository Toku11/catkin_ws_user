#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <iostream>
#include "sensor_msgs/Joy.h"
#include <tf/transform_datatypes.h>


double x = 0.0;
double y = 0.0;
double th = 0.0;

double head=0.0;
double headr=0.0;
double last_head=0.0;

double vx = 0.0;
double vxx = 0.0;
double vyy = 0.0;
double vy = 0.0;
double vth = 0.0;
bool init=false;

///////////Added to make filter
float dt;

float pk[4][4]={0.00001,0,0,0,0,0.00001,0,0,0,0,0.00001,0,0,0,0,0.00001};
float pke[4][4]={0.00001,0,0,0,0,0.00001,0,0,0,0,0.00001,0,0,0,0,0.00001};                            
float r[2][2]={0.000000000013088648208576,-0.000000000041719568873392,-0.000000000041719568873392,0.000000000132987646414364};
float rp[2][2]={0.000000000058085946352195,-0.000000000030468940292028,-0.000000000030468940292028,0.000000000146340235760600};


float xest[4],xk[4],pest[4][4],xestA[4],xkA[4],K[4][2];
ros::Time current_time_twist, last_time_twist;
float velx;

float pcamx,pcamy;
bool gps_received=false;

float alfa =0.8;//0.87;
float alfa2=0.5;
float thgps=0;
float thn;
float offset=0,offsetsend=0;;
float alfa3=0.94;//0.96 funciona bien si el gyno no deriva mucho
float newth=0;
float alfa4=0.8;
float alfa5=0.65;
int contador=0;
bool correct=false;

void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg){
  

  /*tf::Quaternion q=msg->pose.pose.orientation();
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);*/
  pcamx=msg->pose.pose.position.x;
  pcamy=msg->pose.pose.position.y;
  thgps=msg->pose.pose.orientation.z;
  offset=alfa3*offset+(1-alfa3)*msg->twist.twist.angular.z;
  //newth=alfa5*newth+(1-alfa5)*thcam;
  //////////if(offset>3||offset<-3){
  //offsetsend+=offset;
  //offset=0;
  //}
  if (gps_received==false){
    gps_received=true;
  }
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    if(joy->axes[5]>=0.5){
      //correct=true;
    }
    
  }


void twistCallback(const geometry_msgs::Twist& msg)
{
  if(msg.linear.x<=1&&msg.linear.x>=-1){
    velx=0;
  }else{
    velx=msg.linear.x;
  }
  //float vx_=msg.linear.x / (9.54929658551*6.0)*(0.031);///round(msg.linear.x / (9.54929659643*6.0))*(0.031);// 9.54929659643 rpm = 1rad/s and gear ratio: 6  and the wheel Radius 0.31 meter
  float vx_=round(velx / (9.54929658551*6.0))*(0.031);// 9.54929659643 rpm = 1rad/s and gear ratio: 6  and the wheel Radius 0.31 meter	
  vx = roundf(vx_ * -10000) / 10000;  /* Result: 37.78 */
}
void headingCallback(const std_msgs::Float32& msg)
{
  if (init==false)
  {
    init=true;
    head=msg.data* (3.14/180.0); //rad
    last_head=head;
    vth=0.0;
    current_time_twist = ros::Time::now();
    last_time_twist=current_time_twist;
    th=msg.data* (3.14/180.0);
  }
  else
  {
    last_time_twist=current_time_twist;
    current_time_twist = ros::Time::now();
    last_head=head;
    
    //newth=alfa4*newth+(1-alfa4)*head;
   // head=msg.data;
    head=msg.data+offset;//+offset; //rad
    if (head>180)
     head=head-360;
    else if (head<-180.0)
      head=head+360;
    double dt_twist = (current_time_twist - last_time_twist).toSec();
    double delta_head=head-last_head;
    if (delta_head>180)
     delta_head=delta_head-360;
    else if (delta_head<-180.0)
      delta_head=delta_head+360;
    float vth_=0.0;
    vth_= delta_head* (3.14/180.0)/dt_twist;
    headr=head* (3.14159/180.0); //rad
    vth = roundf(vth_ * 10000) / 10000;  /* Result: 37.78 */
    th = roundf(headr * 10000) / 10000;
  }
  
}
int main(int argc, char** argv){



  ros::init(argc, argv, "odometry_usr_node");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 2);

  ros::Subscriber twist_sub = n.subscribe( "motor_control/twist", 1, twistCallback);
  ros::Subscriber theta_sub = n.subscribe( "model_car/yaw2", 1, headingCallback);//degree
  ros::Subscriber visual_sub = n.subscribe( "gps_odometry", 1, gpsCallback);//degree
  ros::Subscriber joy_subscriber = n.subscribe("joy", 1, joyCallback);

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate ra(100.0);
  //ros::Rate r(100.0);
  xk[0]=1.05;
  xk[1]=0;
   ROS_INFO("Corre el de nosotros");
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).toSec();
    dt = roundf(dt * 100000) / 100000;
    //double vxx = -vx * sin(th);//los acablie cos por sen para que coincidan con la camara//vx * cos(th); //- vy * sin(th)) * dt;
    //double vyy = vx * cos(th);//los acablie sin por cos para que coincidan con la camara
    double vxx = -vx * sin(th);//los acablie cos por sen para que coincidan con la camara//vx * cos(th); //- vy * sin(th)) * dt;
    double vyy = vx * cos(th);//los acablie sin por cos para que coincidan con la camara
    double delta_th = vth * dt*180/3.14159;
    //thn+=delta_th;
    if(gps_received==false){
      //xk[2]=vxx;
      //xk[3]=vyy;
      xk[0]=xk[0]+vxx*dt;//como el mejor estimado cuando no hay correccion es xk por eso pongo de una vez xk en vez de xest
      xk[1]=xk[1]+vyy*dt;//


    }else{//se recibieron datos de odometria visual
      gps_received=false;
      xest[0]=xk[0]+vxx*dt;//-dt*xk[2]+vxx*dt;//como el mejor estimado cuando no hay correccion es xk por eso pongo de una vez xk en vez de xest
      xest[1]=xk[1]+vyy*dt;//-dt*xk[3]+vyy*dt;//

      xk[0]=alfa*xest[0] + (1-alfa)*pcamx;
      xk[1]=alfa*xest[1] + (1-alfa)*pcamy;


    }

    //double delta_x = vxx*dt; //- vy * sin(th)) * dt; //calulo simple de odometria
    //double delta_y = vyy*dt; //+ vy * cos * dt;//calculo simple de  odometria

    //double delta_th = vth * dt;//0.01 ; //* dt;
    //x += delta_x;
    //y += delta_y;
    //th += delta_th;

    ///ROS_INFO("t %0.6f",dt);
    //ROS_INFO("v %0.5f",vx);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    //geometry_msgs::TransformStamped odom_trans;
    //odom_trans.header.stamp = current_time;
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = "base_link";

    //odom_trans.transform.translation.x = x;
    //odom_trans.transform.translation.y = y;
    //odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = xk[0];//x;
    odom.pose.pose.position.y = xk[1];//y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = th*180/3.14159;
    //odom.twist.twist.linear.z= offset;
    odom.twist.twist.angular.x=thgps;
    odom.twist.twist.angular.y=th*180/3.14159;//offsetsend;
    odom.twist.twist.angular.z = offset;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ra.sleep();
  }
}
