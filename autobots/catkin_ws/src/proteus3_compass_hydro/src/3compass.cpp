/**
 * This ROS node does the following:
 *
 * 1. Reads in compass data from a serial port
 * 2. Packages up the data within a message
 * 3. Publishes the message through ROS topic /compass/measurement
 *
 * The compass is an HMC6343 with Tilt Compensation device.
 * Thus it reports the heading, pitch, and roll.
 *
 * @author Chien-Liang Fok
 */
#include <string>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include "serial/serial.h"

#include "ros/ros.h"
#include "proteus3_compass_hydro/CompassMsg.h"
#include <sstream>

#define PROTEUS_START 0x24 // a special byte to indicate the start of a message
#define COMPASS_MESSAGE_SIZE 8 // the size of the compass message in bytes

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

int run(int argc, char **argv) {
  ros::init(argc, argv, "compass");

  // This node handle's namespace is "compass".
  // See:  http://www.ros.org/wiki/roscpp/Overview/NodeHandles#Namespaces
  // IMPORTANT:  the correct syntax to publish to the compass namespace is ->  ros::NodeHandle node("compass")
  //      This was changed by Pedro Santacruz on 3/18/2014
  ros::NodeHandle node("compass");

  // Get the parameters
  // See: http://www.ros.org/wiki/roscpp/Overview/Parameter%20Server
  std::string port;
  node.param<std::string>("port", port, "/dev/ttyACM0");
  cout << "Port: " << port << endl;

  int baud = 115200;
  node.param<int>("baud", baud, 115200);
  cout << "Baud: " << baud << endl;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  /*
   * Tell ROS that this node is going to publish messages on topic "measurement".
   * The buffer size is 1000, meaning up to 1000 messages will be stored
   * before throwing any away.
   */
  ros::Publisher chatter_pub = node.advertise<proteus3_compass_hydro::CompassMsg>("measurement", 1000);
  /*
   * Loop at 20Hz.
   */
  ros::Rate loop_rate(20);
  uint8_t *buff = new uint8_t[COMPASS_MESSAGE_SIZE];
  bool readMsg = false;

  while (ros::ok()) {
    if (my_serial.available() >= COMPASS_MESSAGE_SIZE) {
      /* Read one byte from the serial port.
       * If it is a PROTEUS_START byte, read
       * the remaining bytes in the message
       * then process the message.
       */


      size_t numBytes = my_serial.read(buff, 1);
      if (numBytes == 1) {
        if (buff[0] == PROTEUS_START) {
          readMsg = true;
          numBytes = my_serial.read(&buff[1], COMPASS_MESSAGE_SIZE - 1);

          if (numBytes == COMPASS_MESSAGE_SIZE - 1) {

            // print the message
            /*printf("Read message: ");
            for (uint8_t i = 0; i < COMPASS_MESSAGE_SIZE; i++) {
              printf("0x%x ", buff[i]);
            }
            printf("\n");*/

            /* Compute the checksum, which is the XOR of all the bytes
             * in the message except for the last one
             */
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < COMPASS_MESSAGE_SIZE - 1; i++) {
              checksum ^= buff[i];
            }
            //printf("Checksum = 0x%x\n", checksum);

            if (checksum == buff[19]) {
              //printf("Checksum matched!\n");
              float headingDeg1 = ((uint16_t)((buff[1] << 8) + buff[2])) / 10.0; // heading in degrees (0 to 360)
              float headingDeg2 = ((uint16_t)((buff[7] << 8) + buff[8])) / 10.0; // pitch in degrees (-90 to 90)
              float headingDeg3 =  ((uint16_t)((buff[13] << 8) + buff[14])) / 10.0; // roll in degrees (-90 to 90)
              float pitchDeg = ((int16_t)((buff[14] << 8) + buff[15])) / 10.0; // pitch in degrees (-90 to 90)
              float rollDeg =  ((int16_t)((buff[16] << 8) + buff[17])) / 10.0; // roll in degrees (-90 to 90)
              float heading [3] = {headingDeg1,headingDeg2,headingDeg3};
			  float heading_var[3];
              // From the way the compass is mounted on the Proteus III,
              // the heading measurement needs to be rotated by -90 degrees.
              cout << "Heading1: " << headingDeg1 <<endl;
              cout << "Heading2: " << headingDeg2 <<endl;
              cout << "Heading3: " << headingDeg3 <<endl;
			  
			  float headingMean = (headingDeg+headingDeg2+headingDeg3)/3;
			  heading_var[0] = heading[0]-headingMean;
			  heading_var[1] = heading[1]-headingMean;
			  heading_var[2] = heading[2]-headingMean;
              for(int i =0; i<3; i++)
			  {
				  if(heading_var[i] <0)
					  heading_var[i] *=-1;
			  }
			  float remove = heading_var[0] < heading_var[1] ? heading_var[1] : heading_var[0];
			  remove = remove < heading_var[2] ? heading_var[2] : remove;
			  float headingDeg;
			  if(remove == heading_var[0])
				headingDeg = (heading[2] + heading[1])/2;
			  else if (remove == heading[1])
				headingDeg = (heading[0] + heading[2])/2;
			  else
				headingDeg = (heading[0]+heading[1])/2;
			 // From the way the compass is mounted on the Proteus III,
              // the heading measurement needs to be rotated by -90 degrees.
              /*headingDeg -= 90;
              if (headingDeg < 0)
                headingDeg = 360 + headingDeg;

              // Convert the heading from 0-360 to -180 to 180
              if (headingDeg < 180)
                headingDeg *= -1;
              else
                headingDeg = 360 - headingDeg; //180 - (headingDeg - 180)
				*/
				headingDeg -= 180;

			 cout << "Heading: " << headingDeg << ", Pitch: " << pitchDeg << ", Roll: " << rollDeg << endl;
            //  cout <<  headingDeg <<  pitchDeg <<  rollDeg << endl;


              proteus3_compass_hydro::CompassMsg msg;
              msg.heading = headingDeg;
              msg.pitch = pitchDeg;
              msg.roll = rollDeg;

              //ROS_INFO("%s", msg.data.c_str());

              chatter_pub.publish(msg);

            } else {
              printf("Checksum missmatch! (0x%x != 0x%x)\n", checksum, buff[7]);
            }
          } else {
            printf("Failed to read entire message (%lu of %i bytes)\n", numBytes, COMPASS_MESSAGE_SIZE - 1);
          }
        } else {
          printf("First byte not start byte, discard it...\n");
          readMsg = false;
        }
      }
    }

    ros::spinOnce();

    // only sleep if a message was successfully read
    if (readMsg)
      loop_rate.sleep();
  }

  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
  return 0;
}
