/**
 * This ROS node does the following:
 *
 * 1. Reads in GPS data from a serial port
 * 2. Saves the data within a GPS message
 * 3. Publishes the message through ROS topic /gps/measurement
 *
 * This node was written for the LS23060 GPS receiver, which generates NMEA sentences.
 * Here is the receiver's datasheet:
 *
 * http://pharos.ece.utexas.edu/wiki/images/f/fa/LS20030~3_datasheet_v1.2.pdf 
 *
 * Some of the code below was taken from the Player GPS driver
 * under server/drivers/gps/garminnmea.cc available here:
 * http://playerstage.svn.sourceforge.net/viewvc/playerstage/code/player/trunk/server/drivers/gps/garminnmea.cc?revision=9100&content-type=text%2Fplain
 *
 * @author Chien-Liang Fok
 */

#include <string>
#include <iostream>
#include <cstdio>
#include <vector>
#include <sstream>
#include <time.h>

#include "serial/serial.h"

#include "ros/ros.h"
#include "proteus3_gps_hydro/GPSMsg.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

#define NMEA_GPGGA "$GPGGA"
#define NMEA_GPRMC "$GPRMC"

/* WGS84 Parameters*/
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity

/* UTM Parameters*/
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0			// false northing on north hemisphere
#define UTM_FN_S	10000000.0		// false northing on south hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2

/* Filtered GPS geodetic coords; for outlier rejection*/
double filter_a, filter_thresh;
double filter_lat, filter_lon;
bool filter_good;

/* This is the message that is published*/
proteus3_gps_hydro::GPSMsg msg;

// Holds the date and time.  The date is obtained through RMC messages, while the time is
// obtained through GGA messages.
struct tm tms;

/**
 * A helper method for splitting a string into tokens.
 */
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/**
 * Splits a string into tokens.
 *
 * @param s The original string.
 * @param delim The token delimeter
 */
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

/*
 * Utility functions to convert geodetic to UTM position
 */
void computeUTM(double lat, double lon, double *x, double *y) {
	// constants
	const static double m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256);
	const static double m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024);
	const static double m2 = (15*UTM_E4/256 + 45*UTM_E6/1024);
	const static double m3 = -(35*UTM_E6/3072);

	// compute the central meridian
	int cm = (lon >= 0.0) ? ((int)lon - ((int)lon)%6 + 3) : ((int)lon - ((int)lon)%6 - 3);

	// convert degrees into radians
	double rlat = lat * M_PI/180;
	double rlon = lon * M_PI/180;
	double rlon0 = cm * M_PI/180;

	// compute trigonometric functions
	double slat = sin(rlat);
	double clat = cos(rlat);
	double tlat = tan(rlat);

	// decide the flase northing at origin
	double fn = (lat > 0) ? UTM_FN_N : UTM_FN_S;

	double T = tlat * tlat;
	double C = UTM_EP2 * clat * clat;
	double A = (rlon - rlon0) * clat;
	double M = WGS84_A * (m0*rlat + m1*sin(2*rlat) + m2*sin(4*rlat) + m3*sin(6*rlat));
	double V = WGS84_A / sqrt(1 - UTM_E2*slat*slat);

	// compute the easting-northing coordinates
	*x = UTM_FE + UTM_K0 * V * (A + (1-T+C)*pow(A,3)/6 + (5-18*T+T*T+72*C-58*UTM_EP2)*pow(A,5)/120);
	*y = fn + UTM_K0 * (M + V * tlat * (A*A/2 + (5-T+9*C+4*C*C)*pow(A,4)/24 + (61-58*T+T*T+600*C-330*UTM_EP2)*pow(A,6)/720));

  return;
}

/**
 * Parses a GPGGA sentence. This contains the GPS location.
 *
 * @return 0 if successful, -1 otherwise.
 */
int parseGPGGA(const std::vector<std::string> tokens) {
  /*cout << "parseGPGGA called with the following tokens:" << endl;
  // Print the tokens (for debugging)
  for(std::vector<int>::size_type i = 0; i != tokens.size(); i++) {
    std::cout << "\t" << i << "\t" << tokens[i] << endl;
  }*/
 
  if (tokens.size() < 15) {
    std::cout << "WARNING: GPGGA sentence too short, rejecting" << endl;
    return -1;
  }
  if (tokens.size() > 15) {
    std::cout << "WARNING: GPGGA sentence too long, rejecting" << endl;
    return -1;
  }
  if (tokens[3].length() != 1 || (tokens[3][0] != 'N' && tokens[3][0] != 'S')) {
    std::cout << "WARNING: GPGGA sentence rejected b/c invalid N/S designator" << endl;
    return -1;
  }
  if (tokens[5].length() != 1 || (tokens[5][0] != 'E' && tokens[5][0] != 'W')) {
    std::cout << "WARNING: GPGGA sentence rejected b/c invalid E/W designator" << endl;
    return -1;
  }


  char tmp[8]; // A temporary working buffer for holding token fragments
  double degrees, minutes, arcseconds;
  double lat, lon;
  double utm_e, utm_n;

  // Field 1 is UTC Time in hhmmss.sss
  if (tokens[1].length() != 10) { 
    std::cout << "WARNING: GPGGA setence rejected b/c invalid UTC time" << endl;
    return -1;
  }

  tokens[1].copy(tmp, 2); // First two characters is the hours
  tmp[2]='\0';
  tms.tm_hour = atoi(tmp);

  tokens[1].copy(tmp, 2, 2); // Second two characters is the minutes
  tmp[2]='\0';
  tms.tm_min = atoi(tmp);

  tokens[1].copy(tmp, 2, 4); // Fourth two characters is the integral seconds
  tmp[2] = '\0';
  tms.tm_sec = atoi(tmp);

  tokens[1].copy(tmp, 3, 7); // Remaining characters after the period is the milliseconds
  tmp[3] = '\0'; 
  int milliseconds = atoi(tmp);

  // Field 2 is the Latitude in ddmm.mmmm
  if (tokens[2].length() != 9 || tokens[2][4] != '.') {
    std::cout << "WARNING: GPGGA setence rejected b/c invalid latitude: " << tokens[2] << endl;
    return -1;
  }
  tokens[2].copy(tmp, 2); 
  tmp[2]='\0';
  degrees = atoi(tmp);
  minutes = atof(tokens[2].substr(2).c_str());
  arcseconds = ((degrees * 60.0) + minutes) * 60.0;

  // Field 3 is 'N' or 'S' for north or south. Adjust sign accordingly.
  if(tokens[3][0] == 'S')
    arcseconds *= -1;

  lat = arcseconds / 3600.0;
  //cout << "\tlatitude: " << lat << endl;

  // Field 4 is the Longitude in dddmm.mmmm
  if (tokens[4].length() != 10 || tokens[4][5] != '.') {
    std::cout << "WARNING: GPGGA setence rejected b/c invalid longitude: " << tokens[4] << endl;
    return -1;
  }

  tokens[4].copy(tmp, 3);
  tmp[3]='\0';
  degrees = atoi(tmp);
  minutes = atof(tokens[4].substr(3).c_str());
  arcseconds = ((degrees * 60.0) + minutes) * 60.0;

  // Field 5 is 'E' or 'W' for east or west. Adjust sign accordingly.
  if(tokens[5][0] == 'W')
    arcseconds *= -1;

  lon = arcseconds / 3600.0;
  //cout << "\tlongitude: " << lon << endl;
 
  // Field 6 is the fix indicator
  int fixInd = atoi(tokens[6].c_str());
  //cout << "\tPosition fix indicator: " << fixInd << endl;
 
  // Field 7 is the number of satellites used
  int numSats = atoi(tokens[7].c_str());
  //cout << "\tNum satellites: " << numSats << endl;

  // Field 8 is the HDOP (Horizontal Dilution of Precision)
  float hdop = atof(tokens[8].c_str());
  //cout << "\tHDOP: " << hdop << endl;

  // Field 9 is the altitude in meters
  float altitude = atof(tokens[9].c_str());
  //cout << "\tAltitude: " << altitude << endl;

  // Field 10 is the altitude's reference point, e.g., 'M' is
  // mean sea level.  Ignore it.

  // Field 11 is "geoid separation".  Ignore it.

  // Field 12 is the reference point for the above geoid separation.  Ignore it.

  // Field 13 is the differential GPS reference station ID.  Ignore it.

  // Field 14 is the checksum.  Ignore it.

  // Update the filtered lat/lon and see if the new values are any good
  filter_lat = filter_a * lat + (1 - filter_a) * filter_lat;
  filter_lon = filter_a * lon + (1 - filter_a) * filter_lon;

  // Reject outliers
  filter_good = true;
  if (fabs(lat - filter_lat) > filter_thresh)
    filter_good = false;
  if (fabs(lon - filter_lon) > filter_thresh)
    filter_good = false;

  if (!filter_good) {
    printf("Rejected: (%f, %f), expected (%f, %f)\n", lat, lon, filter_lat, filter_lon);
    return -1;
  } else {

    // Compute the UTM coordindates
    computeUTM(lat, lon, &utm_e, &utm_n);
    //printf("\tutm: %.3f %.3f\n", utm_e, utm_n);


    // Compute time since the epoch.
    //cout << "timezone = " << timezone << endl; 
    time_t utc = mktime(&tms) - timezone; // timezone is defined in time.h  It is needed since mktime assumes &tms is in the local timezone instead of UTC
 
    msg.time_sec = (uint32_t) utc;
    msg.time_usec = (uint32_t) (milliseconds * 1000);

    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = altitude;
    msg.utm_e = utm_e;
    msg.utm_n = utm_n;
    msg.quality = fixInd;
    msg.num_sats = numSats;
    msg.hdop = hdop;

    return 0;
  }
}

/**
 * Parses a GPRMC sentence. This contains the date and time.
 */
void parseGPRMC(const std::vector<std::string> tokens) {
  /*cout << "parseGPRMC called with the following tokens:" << endl;
  // Print the tokens (for debugging)
  for(std::vector<int>::size_type i = 0; i != tokens.size(); i++) {
    std::cout << "\t" << i << "\t" << tokens[i] << endl;
  }*/

  if (tokens.size() < 10) {
    std::cout << "WARNING: GPRMC message too short, rejecting it" << endl;
    return;
  }

  char tmp[8]; // A temporary working buffer for holding token fragments

  memset(&tms, 0, sizeof(tms));

  // Field 1 is UTC Time in hhmmss.sss
  /*if (tokens[1].length() < 6) { 
    // This can happen while indoors.
    return;
  }

  tokens[1].copy(tmp, 2); // First two characters is the hours
  tmp[2]='\0';
  tms.tm_hour = atoi(tmp);

  tokens[1].copy(tmp, 2, 2); // Second two characters is the minutes
  tmp[2]='\0';
  tms.tm_min = atoi(tmp);

  tokens[1].copy(tmp, 2, 4); // Fourth two characters is the integral seconds
  tmp[2] = '\0';
  tms.tm_sec = atoi(tmp);

  tokens[1].copy(tmp, 3, 7);
  tmp[3] = '\0'; 
  int milliseconds = atoi(tmp);
  */
  //cout << "hours: " << tms.tm_hour << ", minutes: " << tms.tm_min << ", seconds: " << tms.tm_sec << ", milliseconds: " << milliseconds << endl;

  msg.heading = atof(tokens[8].c_str());
  // Field 9 is the date in DDMMYY format
  if (tokens[9].length() != 6) {
    // short date field, ignore
    return;
  }
  
  tokens[9].copy(tmp, 2); // First two characters is the day
  tmp[2] = '\0';
  tms.tm_mday = atoi(tmp);

  tokens[9].copy(tmp, 2, 2); // Second two characters is the month
  tmp[2] = '\0';
  tms.tm_mon = atoi(tmp) - 1; // months since January (0-11)

  tokens[9].copy(tmp, 2, 4); // Fourth two characters is the year
  tmp[2] = '\0';
  tms.tm_year = 100 + atoi(tmp); // years since 1900

  /*printf("%02d %02d %02d : %02d %02d %02d \n",
         tms.tm_year, tms.tm_mon, tms.tm_mday,
         tms.tm_hour, tms.tm_min, tms.tm_sec);*/

  // Compute time since the epoch. 
  /*utc = mktime(&tms);

  msg.time_sec = (uint32_t) utc;
  msg.time_usec = (uint32_t) (milliseconds * 1000); */ 
}

/**
 * The main run method that contains the loop that periodically
 * reads in the sentences created by the GPS receiver.
 *
 * @param argc The number of arguments
 * @param argv The arguments
 */
int run(int argc, char **argv) {
  ros::init(argc, argv, "gps");

  // This node handle's namespace is "gps".
  // See:  http://www.ros.org/wiki/roscpp/Overview/NodeHandles#Namespaces
  // IMPORTANT:  the correct syntax to publish to the gps namespace is ->  ros::NodeHandle node("gps")
  //      This was changed by Pedro Santacruz on 3/18/2014
  ros::NodeHandle node("gps");

  // Get the parameters
  // See: http://www.ros.org/wiki/roscpp/Overview/Parameter%20Server
  std::string port;
  node.param<std::string>("port", port, "/dev/ttyUSB1");
  cout << "Port: " << port << endl;
  
  int baud = 57600;
  node.param<int>("baud", baud, 57600);
  cout << "Baud: " << baud << endl;

  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  //Initialize the GPS outlier detection variables
  filter_a = 0.80;
  filter_thresh = 1.0;
  filter_lat = 0;
  filter_lon = 0;

  /*
   * Tell ROS that this node is going to publish messages on topic "measurement".
   * The buffer size is 1000, meaning up to 1000 messages will be stored  
   * before throwing any away.
   */
  ros::Publisher chatter_pub = node.advertise<proteus3_gps_hydro::GPSMsg>("measurement", 1000);
  
  /*
   * Loop at 30Hz.
   */
  ros::Rate loop_rate(30);
  
  //int count = 0;
  //uint8_t *buff = new uint8_t[COMPASS_MESSAGE_SIZE];
  bool readLine = false;

  while (ros::ok()) {
    if (my_serial.available() > 0) {
      string currLine = my_serial.readline(200, "\n");
      cout << "Read line: " << currLine;
      readLine = true;

      std::vector<std::string> tokens = split(currLine, ',');
      
      // Print the tokens (for debugging) 
      //for(std::vector<int>::size_type i = 0; i != tokens.size(); i++) {
      //  std::cout << "\t" << i << "\t" << tokens[i] << endl;
      //}

      if (!tokens[0].compare(NMEA_GPGGA)) {
        if (parseGPGGA(tokens) == 0) {
          //ROS_INFO("Publishing %f", msg.latitude);
          //cout << "Publishing GPS Message!" << endl;
          chatter_pub.publish(msg);
        }
      } else if (!tokens[0].compare(NMEA_GPRMC))
        parseGPRMC(tokens);
    } else {
      readLine = false;
    }

    ros::spinOnce();
    
    // only sleep if a message was successfully read
    if (readLine) 
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
