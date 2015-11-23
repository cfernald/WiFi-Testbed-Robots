/* ROS Node to receive messages from gps.cpp

*/

#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <sstream>
#include <time.h>
#include <fstream>

#include "serial/serial.h"

#include "ros/ros.h"
#include "proteus3_gps_hydro/GPSMsg.h"
#include "proteus3_compass_hydro/CompassMsg.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

float Heading;

std::ofstream ofs;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const proteus3_gps_hydro::GPSMsg::ConstPtr& msg)
{
  ROS_INFO("Latitude: [%f]", msg->latitude);
  ROS_INFO("Longitude: [%f]", msg->longitude);
  ofs << std::setprecision(8) << msg->latitude << "," << msg->longitude << "," << msg->heading << "," << Heading << endl;
}

void chatterCallback2(const proteus3_compass_hydro::CompassMsg::ConstPtr& msg)
{
  Heading = msg->heading;
}

bool file_exists (const std::string& name) {
    std::ifstream f(name.c_str());
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }   
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "navigation_plot");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle node;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = node.subscribe("gps/measurement", 1000, chatterCallback);
  ros::Subscriber sub2 = node.subscribe("compass/measurement", 1000, chatterCallback2);

  ROS_INFO("SUBSCRIBED");
  string username(getenv ("USER"));
  string fileaddress = "/home/"+ username + "/ros_outdoor_navigation_data/results_";
 
  for(int i=0; i<10; i++) {
     std::ostringstream s;
     s << fileaddress << i << ".csv";
     string complete_address (s.str());
     if (!file_exists(complete_address)) {
       ofs.open (complete_address.c_str(), std::ofstream::out | std::ofstream::app);
       ROS_INFO("Created the data file at: %s", complete_address.c_str() );
       break;
     } 
  }
  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}
