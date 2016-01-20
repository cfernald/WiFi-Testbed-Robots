/**
 * This is the firmware for the Arduino that controls the 
 * Lynx Motion Pan and Tilt Assembly.
 *
 * Pin assignments:
 *  - Pin 8: Tilt
 *  - Pin 9: Pan
 *
 * Pan/tilt command message:
 * [PROTEUS_BEGIN] [PAN ANGLE] [TILT ANGLE] [CHECKSUM]
 *
 * @author Chien-Liang Fok
 * @date 02/21/2012
 */

#include <Servo.h>

#define PROTEUS_BEGIN 0x24

#define SERVO_CENTER 1500
#define SERVO_MIN 500
#define SERVO_MAX 2500

struct PanTiltCmd {
  uint8_t begin;
  uint16_t pan;  // in servo units (500-2500)
  uint16_t tilt; // in servo units (500-2500)
  uint8_t checksum;
} panTiltCmd;

/*
 * Define the pins used by this program.
 */
enum PIN_ASSIGNMENTS {
  SERVO_TILT_PIN = 8,
  SERVO_PAN_PIN = 9,
  LED_CMD_RCVD_PIN = 13, // This is connected to the on-board LED
};

byte _ledStateCmdRcvd = HIGH;
byte* _panTiltCmdBuff = (byte*)&panTiltCmd;

Servo servoPan;
Servo servoTilt;

void setup() {
  // Initialize LED
  pinMode(LED_CMD_RCVD_PIN, OUTPUT);
  
  // Initialize servos
  servoPan.attach(SERVO_PAN_PIN);
  servoTilt.attach(SERVO_TILT_PIN);
  servoPan.writeMicroseconds(SERVO_CENTER);
  servoTilt.writeMicroseconds(SERVO_CENTER);

  // Initialize the serial port.  This is for communicating with the PC.
  Serial.begin(115200);
}

void loop() {
  rcvPanTiltCmd();
}
