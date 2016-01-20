/*******   Implements a ROS node that interfaces with the Traxxas
Stampede mobility plane used by the Pharos Lab at UT Austin, 
together with a razor 9DOF imu and publishes the odometry of the proteus robot.

Author: Siavash Zangeneh
Date: 08/15/2013

*******/

#include "ros/ros.h"                           //roscpp library
#include "traxxas_node/AckermannDriveMsg.h"    //ackermann ros message header
#include <tf/transform_broadcaster.h>          //ros tf library
#include <nav_msgs/Odometry.h>                 //ros odometry library
#include <termios.h>                           // terminal io (serial port) interface
#include <fcntl.h>                             // File control definitions
#include <errno.h>                             // Error number definitions

// Proteus and Razor IMU constant definitions
#define PROTEUS_START 0x24
#define IMU_START 'P'
#define IMU_END '3'

// user variable definitions
typedef int ComPortHandle;
typedef unsigned char Byte;

//Global variables declarations
char read_buffer_traxxas[18];
char read_buffer_imu[36];
char drive_message[7];
int steeringCmd=0;
int speedCmd=0;
ros::Time lastCmd;
ComPortHandle serial_port_imu;
ComPortHandle serial_port_traxxas;
struct termios options_original;

/****
Function to combine two bytes and make a signed short
Inputs: most significant byte, least significant byte
output: short combined integer 
****/
short MakeShort(Byte msb, Byte lsb)
{
        //short must be a 2 byte integer
        assert(sizeof(short) == 2);

        short s = 0;

        //map the short to a byte array
        Byte* tmp = (Byte*)&s;
         tmp[1] = msb;
         tmp[0] = lsb;

        return s;
}

/****
Function to purge (delete all entries) of a serial port)
input: the serial port handler
output: boolean true or false that indicates the success of the function
****/
bool Purge(ComPortHandle comPortHandle)
{
  if (tcflush(comPortHandle,TCIOFLUSH)==-1) {
    ROS_ERROR("Flush failed!");
    return false;
  }
  return true;
}

/****
Function to open a serial port
input 1: a variable that holds the serial port handler (passed by reference)
input 2: the name of the port, type of string
input 3: baud rate, currently supports baud rates of 57600 and 9600. You can edit the function
and add the support of other baud rates. look in termios.h library to see which baud constants are valid.
output: returns the serial port handler number. (redundent because a handler is updated as an input)
****/
int serial_port_open(ComPortHandle& serial_port, std::string port_name, int baud) {
  struct termios options;

  char *port_name_array=new char[port_name.size()+1];
  port_name_array[port_name.size()]=0;
  memcpy(port_name_array,port_name.c_str(),port_name.size());

  serial_port = open(port_name_array, O_RDWR | O_NONBLOCK);

  if (serial_port != -1) {
    ROS_INFO("Serial Port is open");
    tcgetattr(serial_port,&options_original);
    tcgetattr(serial_port, &options);
    //expand the else if statements here to support more baud rates
    if (baud == 57600) {
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
    } else if (baud == 9600) {
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
    } else {
      ROS_ERROR("baud rate is not supported.");
    }
    // Port settings. look into termios.h for details
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;
    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;
    //Set parity to None
    options.c_cflag &=~PARENB;
    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check close_port(int
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input
    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 10;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    //options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_port, TCSANOW, &options);
  } else {
    ROS_ERROR("Unable to open %s", port_name_array);
  }
  return (serial_port);
}

/**** 
Function to write a string to a serial port
input 1: the serial port handler
input 2: a character string holding the output string
input 3: size of the characters to be sent
output: an integer indicating the number of characters written to the port, it's -1 if the write fails.
****/
int serial_port_write(ComPortHandle& serial_port, char *write_buffer, size_t len) {
  int bytes_written;
  bytes_written = write(serial_port, write_buffer, len);
  if (bytes_written < len) {
    ROS_INFO("Write failed \n");
  }
  return bytes_written;
}

/**** 
Function to read a string from a serial port
input 1: the serial port handler
input 2: a character string as a placeholder for the input string
input 3: size of the characters to be read
output: an integer indicating the number of characters read from the port, it's -1 if the read fails.
****/
int serial_port_read(ComPortHandle& serial_port, char *read_buffer, int max_chars_to_read) {
  int chars_read = read(serial_port, read_buffer, max_chars_to_read);
  return chars_read;
}

/****
Function that's going to convert a string of character to a string of floats used to recover numbers from the imu.
This only works if the imu and the computer follow the same endianness. Otherwise, the order of the bytes should be rotated.
input 1: the input character string
input 2: the placeholder for the float array
input 3: number of characters in the character array. (should be multiple of 4)
output: returns the number of float data that are converted
****/
int chars_to_floats(const char *chars, float *floats, int len)
{
  int converted = 0;
  float *fp = (float*)chars;
  while( len >= sizeof*fp )
  {
    *floats++ = *fp++;
    len -= sizeof*fp;
    ++converted;
  }
  return converted;
}

/****
The function that is called everytime a drive message is received.
Input: AckermannDriveMsg
Output: none
Note: The Velocity commands are put into the global variables steeringCmd and speedCmd. lastCmd is updated to current time.
****/
void driveCmdHandler(const traxxas_node::AckermannDriveMsg::ConstPtr& msg)
{
//  ROS_INFO("I heard %f %f",msg->steering_angle,msg->speed);
  steeringCmd = int(msg->steering_angle * 10);      // Convert to 1/10 degrees
  speedCmd = int(msg->speed * 100);                 // Convert to cm/s
  lastCmd = ros::Time::now();
}

int main (int argc, char **argv){
  //varible used in the main program
  std::string port;
  ros::Time now;
  ros::Duration elapsed;
  double elapsed_second;
  int baud;
  int result;
  char temp_read_buffer;
  float read_values[9];
  int first_byte_read_imu = false;
  int first_byte_read_traxxas = false;
  int current_read_traxxas = 0;
  int current_read_imu = 0;
  int return_read_traxxas = 0;
  int return_read_imu = 0;
  short current_speed=0;
  short current_angle=0;
  int checksum;

  //initializing the ros node
  ros::init(argc, argv, "traxxas_driver");
  ros::NodeHandle nh;

  //initializing the usb port connected to the traxxas, the robot should be connected to ttyUSB1
  nh.param<std::string>("/traxxas_node/port", port, "/dev/ttyUSB1");
  port = "/dev/ttyUSB1";
  nh.param<int>("/traxxas_node/baud", baud, 9600);
  serial_port_open(serial_port_traxxas,port,baud);
  //the node will shut down if it cannot connect to the robot
  if (serial_port_traxxas == -1) {
   ROS_ERROR("Shutting down the node .... ");
   ros::shutdown();
   return 0;
  }
  ROS_INFO("Waiting 2s for Arduino to initialize...");
  ros::Duration(2.0).sleep();
  Purge(serial_port_traxxas);

  //the node subscribes to incoming drive commands.
  ros::Subscriber sub = nh.subscribe("traxxas_node/ackermann_drive", 10, driveCmdHandler);

  ROS_INFO("Driver is now running ....");


  //initializing the usb port connected to the imu, the imu should be connected to ttyUSB0
  port = "/dev/ttyUSB0";
  serial_port_open(serial_port_imu, port,57600);
  if (serial_port_imu == -1) {
    ROS_ERROR("Shutting down the node .... ");
    ros::shutdown();
    return 0;
  }
  //set the imu to send binary sensor data
  char SEND_SENSOR_BINARY[6] = "#oscb" ;
  result = serial_port_write(serial_port_imu, SEND_SENSOR_BINARY,5);
  if (result != 5) {
    ROS_ERROR("Couldn't send command to the imu. Shutting down the node .... ");
    ros::shutdown();
    return 0;
  } else {
    ROS_INFO("IMU is set to continuous calibrated binary data.");
  }
  Purge(serial_port_imu);

  //setting the publisher and tf for odometry
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  //variables that hold the current 2d position of the robot
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  //variables used for measuring time difference of the odometry.
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  //setting the frequency of the while loop to 30Hz
  ros::Rate loop_rate(30.0);
  lastCmd = ros::Time::now();
  int count = 0;
  int i=0;
  while (ros::ok())
  {
    //stopping the robot movement if 0.8s is past from the last received velocity command
    now = ros::Time::now();
    elapsed = now - lastCmd;
    elapsed_second = elapsed.toSec();
    if (elapsed_second > 0.8){
      steeringCmd = 0;
      speedCmd = 0;
    }

    //********** sending new velocity commands at 5Hz  ********************
    if (count == 0){
      drive_message[0] = PROTEUS_START;
      drive_message[1] = (steeringCmd&0x0FF);
      drive_message[2] = (steeringCmd&0x00FF00)>>8;
      drive_message[3] = (speedCmd&0x0FF);
      drive_message[4] = (speedCmd&0x00FF00)>>8;
      drive_message[5] = 0;
      drive_message[6] = '\0';


      for (int i=0; i<5;i++){
        drive_message[5] ^= drive_message[i];
      }
      //if (count == 0){
      //ROS_INFO ("0x%x 0x%x %x 0x%x%x 0x%x",drive_message[0],drive_message[1],drive_message[2],
      // drive_message[3],drive_message[4], drive_message[5]);
      //}

      char* S1 = reinterpret_cast<char*>(drive_message);
      //for (int i=0; i<sizeof(MoveCmd);i++){
        //ROS_INFO ("%d: %x ",i,drive_message[i]);
      //}
      serial_port_write(serial_port_traxxas,S1,6);
    }

    //******Reading traxxas feedback at 10Hz, you can uncomment all ROS_INFO commands to see the process **************
    if (count == 0 || count == 3){
      //ROS_INFO("Test!, first = %x", read_buffer_traxxas[0]);
      //Trying to read the start byte if it is not read yet, continues trying until port buffer is empty
      if (first_byte_read_traxxas == false){
        current_read_traxxas = -1;
        return_read_traxxas = 0;
        read_buffer_traxxas[0]=0xFF;
        while (read_buffer_traxxas[0]!=PROTEUS_START && return_read_traxxas>=0){
          return_read_traxxas = serial_port_read(serial_port_traxxas, read_buffer_traxxas, 1);
          //ROS_INFO("reading ... , = %x",read_buffer_traxxas[0]);
          current_read_traxxas = return_read_traxxas;
          if (read_buffer_traxxas[0] == PROTEUS_START){
            current_read_traxxas = 1;
          }
        }
      }
      //Does nothing if the start byte is not received yet
      if (current_read_traxxas<0 && return_read_traxxas<=0) {
        //ROS_INFO("Miss!");
      //Start or Continues reading of the incoming character array
      } else {
        //ROS_INFO("Good!, first = %x", read_buffer_traxxas[0]);
        return_read_traxxas = 1;
        first_byte_read_traxxas=true;
        //tries to read 17 characters sent after the start byte. Stops if the port buffer is empty
        while(current_read_traxxas<18 && return_read_traxxas>0){
          return_read_traxxas = serial_port_read(serial_port_traxxas,&read_buffer_traxxas[current_read_traxxas], 18-current_read_traxxas);
          if (return_read_traxxas > 0){
            current_read_traxxas += return_read_traxxas;
          }
          //ROS_INFO("Read %d, total %d ....., first = %x", return_read_traxxas, current_read_traxxas,read_buffer_traxxas[0]);
        }
        //If all 18 characters are sent, the program will compute and compare the checksum
        if (current_read_traxxas >=18){
          checksum=0;
          first_byte_read_traxxas=false;
          unsigned char* read_buffer_traxxas_temp;
          read_buffer_traxxas_temp = reinterpret_cast<unsigned char*>(read_buffer_traxxas);
          for (int i=0;i<17;i++){
            checksum ^= read_buffer_traxxas_temp[i];
            //ROS_INFO("%d charachter: %X", i,read_buffer[i]);
          }
            //ROS_INFO("%d charachter: %X", 17,read_buffer[17]);
          if ((checksum&0x0FF) == (read_buffer_traxxas_temp[17]&0x0FF)){
            //current_speed and current_angle are updated if the checksum is correct 
            current_speed = (read_buffer_traxxas[3]&0x0FF) + ((read_buffer_traxxas[4]<<8)&0x0FF00);
            current_angle = (read_buffer_traxxas[13]&0x0FF) + ((read_buffer_traxxas[14]<<8)&0x0FF00);
            //ROS_INFO("Passed checksum!, %X, %X, %X, %X, %X, %X, %X, %X",read_buffer_traxxas_temp[0],read_buffer_traxxas_temp[1],read_buffer_traxxas_temp[2],read_buffer_traxxas_temp[3],read_buffer_traxxas_temp[4],read_buffer_traxxas_temp[13],read_buffer_traxxas_temp[14],read_buffer_traxxas_temp[17]);
            //ROS_INFO("Current speed = %d", current_speed);
          } else {
            ROS_ERROR("Failed checksum!, %X, %X, %X, %X, %X, %X, %X, %X",read_buffer_traxxas_temp[0],read_buffer_traxxas_temp[1],read_buffer_traxxas_temp[2],read_buffer_traxxas_temp[3],read_buffer_traxxas_temp[4],read_buffer_traxxas_temp[13],read_buffer_traxxas_temp[14],read_buffer_traxxas_temp[17]);
           // Purge(serial_port);
          }
        } else {
          //ROS_INFO("Not Done Yet!");
        }
      }
    }

   
    //************* Reading IMU data in 30MHz ********************************

      //Reading the start Byte
      if (first_byte_read_imu == false){
        return_read_imu = 0;
        current_read_imu = -1;
        temp_read_buffer=0;
        while (temp_read_buffer != IMU_START && return_read_imu>=0){
          return_read_imu = serial_port_read(serial_port_imu, &temp_read_buffer, 1);
          //ROS_INFO("reading 'S'... ");
          if (temp_read_buffer == IMU_START){
            current_read_imu = 0;
          }
        }
     }
      if (current_read_imu<0 && return_read_imu<=0) {
        //No valid data is received
        //ROS_INFO("Miss!");
      } else {
        //Reading the sensor data
        //ROS_INFO("Good!");
        return_read_imu = 1;
        first_byte_read_imu=true;
        //Trying to read 37 consecutive bytes
        while(current_read_imu<37 && return_read_imu>0){
          return_read_imu = serial_port_read(serial_port_imu, &read_buffer_imu[current_read_imu], 37-current_read_imu);
          if (return_read_imu > 0){
            current_read_imu += return_read_imu;
          }
          //ROS_INFO("Read %d, total %d .....", return_read_imu, current_read_imu);
        }
        if (current_read_imu >=37){
          //It will check the end byte to make sure that the IMU data is correct
          if (read_buffer_imu[36] == IMU_END) {
            //ROS_INFO("End Byte matches ... got the imu data.");
            // If the data is correct, the program will convert the input data to float variables
            chars_to_floats(read_buffer_imu, read_values, 36);
            //ROS_INFO("%f %f %f", read_values[6], read_values[7], read_values[8]);
          } else {
            //It will dismiss the data and outputs an error if the end byte doesn't match
            ROS_ERROR ("End Byte didn't match ... throwing away the input string");
          }
          first_byte_read_imu = false;
        } else {
          //ROS_INFO("Miss!");
        }
      }
    

    // This will update the velocity variables used in odometry. 
    double vx = (current_speed/100.0);
    double vy = 0;
    //double vth = read_values[8];
    double vth = ((current_speed==0||current_angle==0)?0:-read_values[8]);

    //compute odometry in a typical way given the velocities of the robot
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    //ROS_INFO("angular speed: %f, current angle: %f current speed: %d x: %f y: %f",vth ,th, current_speed,x,y);

    //Check for subscribed topics and run again at 30MHz
    ros::spinOnce();
    loop_rate.sleep();
    count = (count+1)%6;

  }
}
