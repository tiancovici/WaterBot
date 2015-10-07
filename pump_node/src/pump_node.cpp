#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "WaterPump.h"

#define STOP 0
#define FORWARD 1
#define REVERSE 2
#define BREAK 3

/* Create water pump class */
WaterPump* pump = new WaterPump();

/* Water pump topic call back */
void pumpCallback(const std_msgs::Int32::ConstPtr& msg)
{
  /* Listen to pump control topic and act accordingly 
     STOP,    0 - Stop water pump completely (less current than break)
     FORWARD, 1 - Move water toward the plant
     REVERSE, 2 - Move water back into the water basin
     BREAK,   3 - Break water pump 
  */
  switch(msg->data){
    case STOP:
      pump->stop();
      break;
    case FORWARD:
       pump->forward();
       break;
    case REVERSE:
       pump->back();
       break;
    case BREAK:
  	   pump->brake();
  	   break;
  }
}

int main(int argc, char **argv)
{
  /* Initialize water pump node */
  ros::init(argc, argv, "pump_node");

  /*  Create node handler to communicate with the ROS System. */
  ros::NodeHandle n;

  /* Listen to pump control topic */
  ros::Subscriber sub = n.subscribe("/pump_control", 1000, pumpCallback);

  /* Let node run indefinitely, till a kill signal (Ctrl-c or master node ends)*/
  ros::spin();

  return 0;
}
