#include <motor_communication/motor_communication.h>

  
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "motor_communication_usr_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  motor_communication_usr node;
  ros::spinOnce();  
  node.init();
  while(ros::ok())
  {
    //node.run(1000);
    ros::spinOnce();  
    loop.sleep();
  }
  node.stop();
  return 0;
}
