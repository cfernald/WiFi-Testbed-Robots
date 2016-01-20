#include "ros/ros.h"
#include "traxxas_node/AckermannDriveMsg.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pthread.h>
#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions


#define PROTEUS_START 0x24


typedef int ComPortHandle;
typedef unsigned char Byte;
/*struct MoveCmd {
  unsigned char begin;
  short steering; // the steering angle in tenths of degrees
  short speed;
  unsigned char checksum;
} moveCmd;
*/
struct EulerAngles {
  float roll;
  float pitch;
  float yaw;
};
struct Accelerations {
  float accel_x;
  float accel_y;
  float accel_z;
};
struct AngRates {
  float angRate_x;
  float angRate_y;
  float angRate_z;
};

unsigned char drive_message[7];
int steeringCmd=0;
int speedCmd=0;
ros::Time lastCmd;
ComPortHandle serial_port;
struct termios options_original;

//Function to combine two bytes and make a signed short
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


bool Purge(ComPortHandle comPortHandle)
{
  if (tcflush(comPortHandle,TCIOFLUSH)==-1) {
    ROS_ERROR("Flush failed!");
    return false;
  }
  return true;
}


int serial_port_open(std::string port_name) {
  struct termios options;

  char *port_name_array=new char[port_name.size()+1];
  port_name_array[port_name.size()]=0;
  memcpy(port_name_array,port_name.c_str(),port_name.size());

  serial_port = open(port_name_array, O_RDWR | O_NONBLOCK);

  if (serial_port != -1)
  {
    ROS_INFO("Serial Port is open");
    tcgetattr(serial_port,&options_original);
    tcgetattr(serial_port, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
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
  //  options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_port, TCSANOW, &options);
  }
  else {
    ROS_ERROR("Unable to open %s", port_name_array);
  }
  return (serial_port);
}

ComPortHandle OpenComPort(const char* comPortPath)
{
  ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY);
  if (comPort== -1) {
    return -1;
  }
  termios options;
  tcgetattr(comPort, &options);
  int baudRate = B38400;
  cfsetospeed(&options, baudRate);
  cfsetispeed(&options, baudRate);
  options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &=~PARENB;
  options.c_iflag = IGNPAR; // ignore parity check close_port(int
  options.c_oflag = 0; // raw output
  options.c_lflag = 0; // raw input
  options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
  options.c_cc[VTIME] = 10;   // Inter-Character Timer -- i.e. timeout= x*.1 s
  options.c_cflag |= (CLOCAL | CREAD);
  Purge(comPort);
  int status=tcsetattr(comPort, TCSANOW, &options);
  if (status != 0) {
    ROS_ERROR( "Configuring IMU port failed. IMU is not functional");
    return status;
  }
  Purge(comPort);
  return comPort;
}

void serial_port_write(char *write_buffer, size_t len) {
  int bytes_written;
  len = 6;
  bytes_written = write(serial_port, write_buffer, len);
  if (bytes_written < len) {
    ROS_INFO("Write failed \n");
  }
}

int serial_port_read(char *read_buffer, int max_chars_to_read) {
  int chars_read = read(serial_port, read_buffer, max_chars_to_read);
  return chars_read;
}

int readComPort(ComPortHandle comPort, Byte* bytes, int bytesToRead) {
  int bytesRead = read(comPort, bytes, bytesToRead);
}

int writeComPort(ComPortHandle comPort, const Byte* bytesToWrite, int size)
{
  return write(comPort, bytesToWrite, size);
}


bool ReadEulerAngles(ComPortHandle comPort, EulerAngles& eulerAngles, Accelerations &accelerations, AngRates& angRates) {
  static const Byte COMMAND_BYTE  = 0x31;
  writeComPort(comPort, &COMMAND_BYTE, 1);
  static const int RESPONSE_SIZE = 23;
  Byte response[RESPONSE_SIZE] = {0};
  int size = readComPort(comPort, &response[0], RESPONSE_SIZE);
  if(size != RESPONSE_SIZE) {
    ROS_ERROR("Invalid IMU response size");
    return false;
  }
  if(response[0] != COMMAND_BYTE) {
    ROS_ERROR("Invalid IMU response");
    return false;
  }  
  //Verify the checksum
  short responseChecksum = MakeShort(response[21], response[22]);
  short calculatedChecksum = COMMAND_BYTE;
  for(int i = 1; i< RESPONSE_SIZE-2; i+=2) {
    calculatedChecksum += MakeShort(response[i], response[i+1]);
  }
  if(calculatedChecksum != responseChecksum) {
    ROS_ERROR("Invalid Checksum");
    return false;
  }
  //conversion factor used to convert the returned values to degrees
  static const float SCALE_AS_DEGREES = 360.0/65536;
  static const float SCALE_AS_MS2 = 32768000/7000;
  static const float SCALE_AS_RADPS = 32768000/46950;
  eulerAngles.roll  = MakeShort(response[1], response[2]) * SCALE_AS_DEGREES;
  eulerAngles.pitch = MakeShort(response[3], response[4]) * SCALE_AS_DEGREES;
  eulerAngles.yaw   = MakeShort(response[5], response[6]) * SCALE_AS_DEGREES;
  accelerations.accel_x = MakeShort(response[7], response[8]) / SCALE_AS_MS2;
  accelerations.accel_y = MakeShort(response[9], response[10]) / SCALE_AS_MS2;
  accelerations.accel_z = MakeShort(response[11], response[12]) / SCALE_AS_MS2;
  angRates.angRate_x = MakeShort(response[13], response[14]) / SCALE_AS_RADPS;
  angRates.angRate_y = MakeShort(response[15], response[16]) / SCALE_AS_RADPS;
  angRates.angRate_z = MakeShort(response[17], response[18]) / SCALE_AS_RADPS;
  return true;
}

bool readStabVectors(ComPortHandle comPort, Accelerations &accelerations, AngRates& angRates) {
  static const Byte COMMAND_BYTE  = 0x03;
  writeComPort(comPort, &COMMAND_BYTE, 1);
  static const int RESPONSE_SIZE = 23;
  Byte response[RESPONSE_SIZE] = {0};
  int size = readComPort(comPort, &response[0], RESPONSE_SIZE);
  if(size != RESPONSE_SIZE) {
    ROS_ERROR("Invalid IMU response size = %d",size);
    return false;
  }
  if(response[0] != COMMAND_BYTE) {
    ROS_ERROR("Invalid IMU response");
    ROS_INFO("The incoming byte is %d", response[0]);
    return false;
  }  
  //Verify the checksum
  short responseChecksum = MakeShort(response[21], response[22]);
  short calculatedChecksum = COMMAND_BYTE;
  for(int i = 1; i< RESPONSE_SIZE-2; i+=2) {
    calculatedChecksum += MakeShort(response[i], response[i+1]);
  }
  if(calculatedChecksum != responseChecksum) {
    ROS_ERROR("Invalid Checksum");
    return false;
  }
  //conversion factor used to convert the returned values to degrees
  //static const float SCALE_AS_DEGREES = 360.0/65536;
  static const float SCALE_AS_MS2 = 32768000/7000;
  static const float SCALE_AS_RADPS = 32768000.0/10000.0;
  //eulerAngles.roll  = MakeShort(response[1], response[2]) * SCALE_AS_DEGREES;
  //eulerAngles.pitch = MakeShort(response[3], response[4]) * SCALE_AS_DEGREES;
  //eulerAngles.yaw   = MakeShort(response[5], response[6]) * SCALE_AS_DEGREES;
  accelerations.accel_x = MakeShort(response[7], response[8]) / SCALE_AS_MS2;
  accelerations.accel_y = MakeShort(response[9], response[10]) / SCALE_AS_MS2;
  accelerations.accel_z = MakeShort(response[11], response[12]) / SCALE_AS_MS2;
  angRates.angRate_x = MakeShort(response[13], response[14]) / SCALE_AS_RADPS;
  angRates.angRate_y = MakeShort(response[15], response[16]) / SCALE_AS_RADPS;
  angRates.angRate_z = MakeShort(response[17], response[18]) / SCALE_AS_RADPS;
  return true;
}

bool setGyroBias(ComPortHandle comPort)
{
  static const Byte command_header = 0x06;
  writeComPort(comPort, &command_header, 1);
  sleep (5.0);
  static const int RESPONSE_SIZE = 5;
  Byte response[RESPONSE_SIZE] = {0};
  int size = readComPort(comPort, &response[0], RESPONSE_SIZE);
  ROS_INFO("read %d", size );
  if(size != RESPONSE_SIZE) {
    ROS_ERROR("Invalid response size");
    return false;
  }
  if(response[0] != command_header) {
    ROS_ERROR("Invalid response");
    return false;
  }
  short responseChecksum = MakeShort(response[3], response[4]);
  short calculatedChecksum = command_header;
  for(int i = 1; i< RESPONSE_SIZE-2; i+=2) {
    calculatedChecksum += MakeShort(response[i], response[i+1]);
  }
  if(calculatedChecksum != responseChecksum) {
    ROS_ERROR("Invalid Checksum" );
    return false;
  }
  return true;
}


void driveCmdHandler(const traxxas_node::AckermannDriveMsg::ConstPtr& msg)
{
//  ROS_INFO("I heard %f %f",msg->steering_angle,msg->speed);
  steeringCmd = int(msg->steering_angle * 10);      // Convert to 1/10 degrees
  speedCmd = int(msg->speed * 100);                 // Convert to cm/s
  lastCmd = ros::Time::now();
}

int main (int argc, char **argv){
  std::string port;
  int baud;
  ros::Time now;
  ros::Duration elapsed;
  double elapsed_second;
  int checksum;
  char read_buffer[18];
  int return_read;
  int current_read;
  short current_speed;
  short current_angle;
  bool first_byte_read= false;
  bool imu_activated = true;
  EulerAngles eulerAngles;
  Accelerations accelerations;
  AngRates angRates;

  ros::init(argc, argv, "traxxas_driver");
  ros::NodeHandle nh;
  nh.param<std::string>("/traxxas_node/port", port, "/dev/ttyUSB0");
  nh.param<int>("/traxxas_node/baud", baud, 9600);

  serial_port_open(port);
  if (serial_port == -1) {
   ROS_ERROR("Shutting down the node .... ");
   ros::shutdown();
   return 0;
  }
  ROS_INFO("Waiting 2s for Arduino to initialize...");
  ros::Duration(2.0).sleep();
  Purge(serial_port);
  ros::Subscriber sub = nh.subscribe("traxxas_node/ackermann_drive", 10, driveCmdHandler);

  ROS_INFO("Driver is now running ....");

  ROS_INFO("Opening comport to the IMU...");
  ComPortHandle comPort = OpenComPort("/dev/ttyS0");
  //ROS_INFO("comPort = %d",comPort);
  if (comPort <= 0) {
    imu_activated = false;
    ROS_ERROR("cannot open the port to connect to the IMU! Odometry transforms will not be published ... ");
  }
  int error_count=5;
  while (error_count>0){
    //if (readStabVectors(comPort,accelerations,angRates)) {
    if ( setGyroBias(comPort)) {
      ros::Duration(2.0).sleep();
      ROS_INFO("IMU is functional.");
      break;
    } else {
      ROS_INFO("Could not connect to IMU. Trying again ... ");
      ros::Duration(0.2).sleep();
      error_count--;
    }
  }
  if (error_count==0){
    ROS_ERROR("The robot could not connect to the IMU to set the gyroscope bias. IMU is not working, thus the angular velocity will not be published!");
    imu_activated = false;
  }

  Purge(comPort);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  ros::Rate loop_rate(30.0);
  lastCmd = ros::Time::now();
  int count = 0;
  int i=0;
  //ROS_INFO("%d %d %d %d %d",sizeof(moveCmd.begin),sizeof(moveCmd.steering),sizeof(moveCmd.speed),sizeof(moveCmd.checksum),sizeof(MoveCmd));
  while (ros::ok())
  {
    now = ros::Time::now();
    elapsed = now - lastCmd;
    elapsed_second = elapsed.toSec();
    if (elapsed_second > 0.8){
      steeringCmd = 0;
      speedCmd = 0;
    }
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
      serial_port_write(S1,6);
    }
    if (count == 0 || count == 3){
      if (first_byte_read == false){
        return_read = 0;
        read_buffer[0]=0xFF;
        while (read_buffer[0]!=PROTEUS_START && return_read>=0){
          return_read = serial_port_read(read_buffer, 1);
          //ROS_INFO("reading ... ");
          current_read = return_read;
        }
      }
      if (current_read<=0 && return_read<=0) {
        //ROS_INFO("Miss!");
      } else {
       // ROS_INFO("Good!");
        return_read = 1;
        first_byte_read=true;
        int error_count=40;
        while(current_read<18 && return_read>0){
          return_read = serial_port_read(&read_buffer[current_read], 18-current_read);
          if (return_read > 0){
            current_read += return_read;
          } else {
            error_count --;
          }
          //ROS_INFO("Read %d, total %d .....", return_read, current_read);
        }
        if (current_read >=18){
          checksum=0;
          first_byte_read=false;
          std::string temp;
          for (int i=0;i<17;i++){
            checksum ^= read_buffer[i];
          //  ROS_INFO("%d charachter: %X", i,read_buffer[i]);
          }
          //  ROS_INFO("%d charachter: %X", 17,read_buffer[17]);
          if ((checksum&0x0FF) == (read_buffer[17]&0x0FF)){
            //ROS_INFO("Passed checksum!");
            current_speed = (read_buffer[3]&0x0FF) + ((read_buffer[4]<<8)&0x0FF00);
            current_angle = (read_buffer[13]&0x0FF) + ((read_buffer[14]<<8)&0x0FF00);
            //ROS_INFO("Current speed = %d", current_speed);
          } else {
            ROS_ERROR("Failed checksum!");
           // Purge(serial_port);
          }
        } else {
          //ROS_INFO("Miss!");
        }
      }
    }
    int j;
    if (imu_activated == true){
      /*j = (j+1)%16;
      if (current_speed == 0) {
        if (j==0){
          setGyroBias(comPort);
        }
      } else if (j>6) {
        readStabVectors(comPort,accelerations,angRates);
        //ROS_INFO("Roll:%0.2f Pitch:%0.2f Yaw:%0.2f\n", eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw);
      }*/
      readStabVectors(comPort,accelerations,angRates);
    }

    current_time = ros::Time::now();

    double vx = current_speed/100.0;
    double vy = 0;
    double vth_bias;
    if (current_speed == 0 || current_angle==0){
      vth_bias = angRates.angRate_z;
    }
    double vth = -(angRates.angRate_z-vth_bias);
    //double vth = (current_speed==0||current_angle==0?0:-angRates.angRate_z);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
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
    /*if (i==20){
      ROS_INFO("Speed: %d Angle: %f AngRate: %f",current_speed,eulerAngles.yaw,angRates.angRate_z);
      ROS_INFO("x: %f  y: %f  th: %f ", x,y,th);
      i=0;
    }*/
    if(i==5){
    ROS_INFO("rate = %f  current = %f",angRates.angRate_z,th);
      //iROS_INFO("Speed: %d AngRate: %f Angle: %d",current_speed,angRates.angRate_z,current_angle);
      //ROS_INFO("x: %f  y: %f  th: %f ", x,y,th);
    i=0;
    }
    i++;

    last_time = current_time;


    ros::spinOnce();
    loop_rate.sleep();
    count = (count+1)%6;

  }
}
