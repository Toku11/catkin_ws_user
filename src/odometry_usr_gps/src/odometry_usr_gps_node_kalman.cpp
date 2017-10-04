#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <iostream>



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
bool visual_received=false;




void visualCallback(const nav_msgs::Odometry::ConstPtr& msg){
  
  pcamx=msg->pose.pose.position.x;
  pcamy=msg->pose.pose.position.y;
  if (visual_received==false){
    visual_received=true;
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
    head=msg.data; //rad
    double dt_twist = (current_time_twist - last_time_twist).toSec();
    double delta_head=head-last_head;
    if (delta_head>180)
     delta_head=delta_head-360;
    else if (delta_head<-180.0)
      delta_head=delta_head+360;
    float vth_=0.0;
    vth_= delta_head* (3.14/180.0)/dt_twist;
    headr=msg.data* (3.14/180.0); //rad
    vth = roundf(vth_ * 10000) / 10000;  /* Result: 37.78 */
    th = roundf(headr * 10000) / 10000;
  }
  
}
int main(int argc, char** argv){



  ros::init(argc, argv, "odometry_usr_node");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 2);

  ros::Subscriber twist_sub = n.subscribe( "motor_control/twist", 2, twistCallback);
  ros::Subscriber theta_sub = n.subscribe( "model_car/yaw", 2, headingCallback);//degree
  ros::Subscriber visual_sub = n.subscribe( "visual_odometry", 1, visualCallback);//degree

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate ra(100.0);
  //ros::Rate r(100.0);
   ROS_INFO("Corre el de nosotros");
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).toSec();
    dt = roundf(dt * 100000) / 100000;
    double vxx = -vx * sin(th);//los acablie cos por sen para que coincidan con la camara//vx * cos(th); //- vy * sin(th)) * dt;
    double vyy = vx * cos(th);//los acablie sin por cos para que coincidan con la camara
    if(visual_received==false){
      xk[2]=vxx;
      xk[3]=vyy;
      xk[0]=xk[0]+vxx*dt;//como el mejor estimado cuando no hay correccion es xk por eso pongo de una vez xk en vez de xest
      xk[1]=xk[1]+vyy*dt;//
      

      pk[0][0]=pk[0][0] - dt*pk[2][0] - dt*(pk[0][2] - dt*pk[2][2]);
      pk[0][1]=pk[0][1] - dt*pk[2][1] - dt*(pk[0][3] - dt*pk[2][3]);
      pk[0][2]=pk[0][2] - dt*pk[2][2];
      pk[0][3]=pk[0][3] - dt*pk[2][3];

      pk[1][0]=pk[1][0] - dt*pk[3][0] - dt*(pk[1][2] - dt*pk[3][2]);
      pk[1][1]=pk[1][1] - dt*pk[3][1] - dt*(pk[1][3] - dt*pk[3][3]);
      pk[1][2]=pk[1][2] - dt*pk[3][2];
      pk[1][3]=pk[1][3] - dt*pk[3][3];

      pk[2][0]=pk[2][0] - dt*pk[2][2];
      pk[2][1]=pk[2][1] - dt*pk[2][3];
      pk[2][2]=pk[2][2] + r[0][0];
      pk[2][3]=pk[2][3] + r[0][1];

      pk[3][0]=pk[3][0] - dt*pk[3][2];
      pk[3][1]=pk[3][1] - dt*pk[3][3];
      pk[3][2]=pk[3][2] + r[1][0];
      pk[3][3]=pk[3][3] + r[1][1]; //los mejores estimados sin tener otros datos de odo


    }else{//se recibieron datos de odometria visual
      visual_received=false;
      xest[0]=xk[0]-dt*xk[2]+vxx*dt;//como el mejor estimado cuando no hay correccion es xk por eso pongo de una vez xk en vez de xest
      xest[1]=xk[1]-dt*xk[3]+vyy*dt;//
      xest[2]=xk[2];
      xest[3]=xk[3];

      pke[0][0]=pk[0][0] - dt*pk[2][0] - dt*(pk[0][2] - dt*pk[2][2]);
      pke[0][1]=pk[0][1] - dt*pk[2][1] - dt*(pk[0][3] - dt*pk[2][3]);
      pke[0][2]=pk[0][2] - dt*pk[2][2];
      pke[0][3]=pk[0][3] - dt*pk[2][3];

      pke[1][0]=pk[1][0] - dt*pk[3][0] - dt*(pk[1][2] - dt*pk[3][2]);
      pke[1][1]=pk[1][1] - dt*pk[3][1] - dt*(pk[1][3] - dt*pk[3][3]);
      pke[1][2]=pk[1][2] - dt*pk[3][2];
      pke[1][3]=pk[1][3] - dt*pk[3][3];

      pke[2][0]=pk[2][0] - dt*pk[2][2];
      pke[2][1]=pk[2][1] - dt*pk[2][3];
      pke[2][2]=pk[2][2] + r[0][0];
      pke[2][3]=pk[2][3] + r[0][1];

      pke[3][0]=pk[3][0] - dt*pk[3][2];
      pke[3][1]=pk[3][1] - dt*pk[3][3];
      pke[3][2]=pk[3][2] + r[1][0];
      pke[3][3]=pk[3][3] + r[1][1];


      K[0][0]=(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);
      K[0][1]=-(pke[0][0]*rp[0][1] - pke[0][1]*rp[0][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);

      K[1][0]=(pke[1][0]*rp[1][1] - pke[1][1]*rp[1][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);
      K[1][1]=(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);

      K[2][0]=-(pke[1][0]*pke[2][1] - pke[1][1]*pke[2][0] - pke[2][0]*rp[1][1] + pke[2][1]*rp[1][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);
      K[2][1]=(pke[0][0]*pke[2][1] - pke[0][1]*pke[2][0] - pke[2][0]*rp[0][1] + pke[2][1]*rp[0][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);

      K[3][0]=-(pke[1][0]*pke[3][1] - pke[1][1]*pke[3][0] - pke[3][0]*rp[1][1] + pke[3][1]*rp[1][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);
      K[3][1]=(pke[0][0]*pke[3][1] - pke[0][1]*pke[3][0] - pke[3][0]*rp[0][1] + pke[3][1]*rp[0][0])/(pke[0][0]*pke[1][1] - pke[0][1]*pke[1][0] + pke[0][0]*rp[1][1] - pke[0][1]*rp[1][0] - pke[1][0]*rp[0][1] + pke[1][1]*rp[0][0] + rp[0][0]*rp[1][1] - rp[0][1]*rp[1][0]);

      pk[0][0]=- K[0][1]*pke[1][0] - pke[0][0]*(K[0][0] - 1);
      pk[0][1]=- K[0][1]*pke[1][1] - pke[0][1]*(K[0][0] - 1);
      pk[0][2]=- K[0][1]*pke[1][2] - pke[0][2]*(K[0][0] - 1);
      pk[0][3]=- K[0][1]*pke[1][3] - pke[0][3]*(K[0][0] - 1);

      pk[1][0]=- K[1][0]*pke[0][0] - pke[1][0]*(K[1][1] - 1);
      pk[1][1]=- K[1][0]*pke[0][1] - pke[1][1]*(K[1][1] - 1);
      pk[1][2]=- K[1][0]*pke[0][2] - pke[1][2]*(K[1][1] - 1);
      pk[1][3]=- K[1][0]*pke[0][3] - pke[1][3]*(K[1][1] - 1);

      pk[2][0]=pke[2][0] - K[2][0]*pke[0][0] - K[2][1]*pke[1][0];
      pk[2][1]=pke[2][1] - K[2][0]*pke[0][1] - K[2][1]*pke[1][1];
      pk[2][2]=pke[2][2] - K[2][0]*pke[0][2] - K[2][1]*pke[1][2];
      pk[2][3]=pke[2][3] - K[2][0]*pke[0][3] - K[2][1]*pke[1][3];

      pk[3][0]=pke[3][0] - K[3][0]*pke[0][0] - K[3][1]*pke[1][0];
      pk[3][1]=pke[3][1] - K[3][0]*pke[0][1] - K[3][1]*pke[1][1];
      pk[3][2]=pke[3][2] - K[3][0]*pke[0][2] - K[3][1]*pke[1][2];
      pk[3][3]=pke[3][3] - K[3][0]*pke[0][3] - K[3][1]*pke[1][3];

      xk[0]=xest[0] + K[0][0]*(pcamx - xest[0]) - K[0][1]*(xest[1] - pcamy);
      xk[1]=xest[1] + K[1][0]*(pcamx - xest[0]) - K[1][1]*(xest[1] - pcamy);
      xk[2]=xest[2] + K[2][0]*(pcamx - xest[0]) - K[2][1]*(xest[1] - pcamy);
      xk[3]=xest[3] + K[3][0]*(pcamx - xest[0]) - K[3][1]*(xest[1] - pcamy);

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
    odom.twist.twist.linear.x = vxx;
    odom.twist.twist.linear.y = vyy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ra.sleep();
  }
}
