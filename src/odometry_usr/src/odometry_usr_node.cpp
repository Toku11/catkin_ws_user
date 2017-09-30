#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>



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
ros::Time current_time_twist, last_time_twist;
float velx;
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

  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  //ros::Rate r(100.0);
   ROS_INFO("Corre el de nosotros");
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    dt = roundf(dt * 100000) / 100000;
    double vxx = vx * cos(th); //- vy * sin(th)) * dt;
    double vyy = vx * sin(th);
    double delta_x = vxx*dt; //- vy * sin(th)) * dt;
    double delta_y = vyy*dt; //+ vy * cos * dt;

    //double delta_th = vth * dt;//0.01 ; //* dt;
    x += delta_x;
    y += delta_y;
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
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = th;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vxx;
    odom.twist.twist.linear.y = vyy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
