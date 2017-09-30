#include <heading/heading.h>


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "heading_node");
  ros::NodeHandle nh;
  heading ReadHeading(nh);
  //os::Rate r(30.0);
   while(ros::ok())
  {
    ReadHeading.getHeading();
    ros::spinOnce();
    //r.sleep();
    
    //ROS_INFO("yaw: %f",yaw);
    
  }
  return 0;
}
