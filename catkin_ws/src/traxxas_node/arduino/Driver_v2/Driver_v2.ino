/**
 * This is the firmware for the Arduino Pro Mini that controls the 
 * Traxxas Stampede mobility plane.
 *
 * Pin assignments:
 *  - Pin 2: Encoder channel A
 *  - Pin 3: Encoder channel B
 *  - Pin 9: Steering servo 
 *  - Pin 10: Motor controller
 *  - Pin 13: LED
 *
 * Drive command message:  The following message must be sent at 10Hz:
 * [PROTEUS_BEGIN] [STEERING ANGLE] [MOTOR SPEED] [CHECKSUM]
 *
 * A status message indicating the steering angle and motor speed
 * is sent back to the x86 at 10Hz.
 *
 * The motor controller is a SyRen 25A.  It is configured to be in packetized
 * serial mode.  DIP switch setting: OFF, OFF, OFF, ON, ON, ON.
 *
 * @author Chien-Liang Fok
 * @date 02/15/2012
 */

#include <SoftwareSerial.h> // for communicating with SyRen 25A motor controller
#include "ProteusServo.h"   // for generating R/C PWM signal for steering servo

#define PROTEUS_BEGIN 0x24

/*
 * Define the range of parameters for controlling the steering angle.
 */
#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

struct MoveCmd {
  uint8_t begin;
  int16_t steering; // the steering angle in tenths of degrees
  int16_t speed;
  uint8_t checksum;
} moveCmd;

struct StatusMsg {
  uint8_t begin;
  int16_t targetSpeed;
  int16_t currSpeed;
  uint16_t motorPwr;
  int16_t prevErr;
  int16_t totalErr;
  int16_t targetSteeringAngle;
  int16_t currSteeringAngle;
  uint16_t steeringAngleCmd;
  uint8_t checksum;
} statusMsg;

/*
 * Define the pins used by this program.
 */
enum PIN_ASSIGNMENTS {
  
  // The following are input pints from the wheel encoder.
  // The wheel encoder pulls these pins high and low in pulses
  // proportional to the number of times the wheel has spun.
  ENCODER_PIN_A = 2,
  ENCODER_PIN_B = 3,
  
  // The following is an input indicating wehther the motor controller is on.
  // When on, the motor controller pulls the pin high (5V)
  MOTOR_STATUS_PIN = 4,
  
  // The Arduino pulls this pin high after it sends the initialization byte
  // to the motor controller.  This pin powers an orange LED on the front
  // panel.
  PIN_ORANGE_LED = 5,
  
  // The following output pin controls a red LED on the front panel.
  // It is blinked whenever the robot enters a safety stop state, which
  // includes when no commands are received over the serial port for an 
  // extended period of time.
  PIN_RED_LED = 6,
  
  // The following output pin controls the direction of the steering servo.
  STEERING_PWM_PIN = 9,
  
  // The following output pin controls the motor controller.
  MOTOR_PIN = 10,
  
  // The following output pin is toggled whenever an ackermann command is received.
  // It is connected to both the green LED on the Arduino and the yellow LED on the
  // front panel connector.
  PIN_YELLOW_LED = 13
};

/*
 * Define constants, variables, and range of parameters used for controlling 
 * the SyRen 25A motor controller.
 */ 
SoftwareSerial _motorPort(11, 10);  // RX, TX

byte _ledStateCmdRcvd = HIGH;
byte* _moveCmdBuff = (byte*)&moveCmd;
byte* _statusMsgBuff = (byte*)&statusMsg;

volatile int _encoderCnt = 0;

boolean _A_set = false;
boolean _B_set = false;

/**
 * Records when the most recent move command was received over the serial port.
 * It is in milliseconds since power on.
 */
unsigned long _prevCmdTime = 0;

int _currSpeed = 0; // in cm/s (50cm/s = 0.5m/s)
int _targetSpeed = 0; // in cm/s (50cm/s = 0.5m/s)
int _throttledTargetSpeed = 0; // in cm/s

int _prevErr = 0;  // The previous error.  This is used when computing the "D" term of the PID controller.
int _totalErr = 0; // The cumulative error since the system started.  This is used by the "I" term in the PID controller

ProteusServo _steeringServo;
int _targetSteeringAngle; // 1/10 degree
int _currSteeringAngle; // 1/10 degree
int _currSteeringAngleCmd = STEERING_CENTER; // servo units
int _prevSteeringAngleCmd = _currSteeringAngleCmd;
// TODO: Add a throttled steering angle that changes the steering angle based on a set rate

boolean _servoDone = false;

/**
 * This is called by ProteusServo.cpp when it is done 
 * updating the servo.
 */
void servoDone() {
  _servoDone = true;
}

void setup() {
  pinMode(PIN_YELLOW_LED, OUTPUT);  // Initialize LED
  
  pinMode(MOTOR_STATUS_PIN, INPUT);
  pinMode(PIN_ORANGE_LED, OUTPUT);
  pinMode(PIN_RED_LED, OUTPUT);
  
  // Configure the encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  digitalWrite(ENCODER_PIN_A, HIGH);  // turn on pullup resistor
  digitalWrite(ENCODER_PIN_B, HIGH);  // turn on pullup resistor

  // Configure the encoder interrupts
  attachInterrupt(0, doEncoderA, CHANGE); // encoder channel A is on pin 2, interrupt 0
  attachInterrupt(1, doEncoderB, CHANGE); // encoder channel B is on pin 3, interrupt 1
 
  // Initialize the motor controller.  It uses a packetized serial protocol
  _motorPort.begin(19200);
  
  _steeringServo.addDoneListener(servoDone);
  _steeringServo.attach(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  _steeringServo.write(STEERING_CENTER);
  
 
  // Initialize the serial port.  This is for communicating the PID controller's state to an attached PC.
  Serial.begin(9600);
}

void loop() {
  
  // Update the status of the LEDs
  updateLEDs();
  
  // Check the status of motor controller.  This determines whether
  // a start byte needs to be sent to the motor controller.
  // It is implemented in MotorControllerInitializer
  checkMotorControllerStatus();
  
  // Read a command sent from the host computer if available
  rcvAckermannCmd();
  
  // Check for the safety stop condition (i.e., when no command is 
  // received for a threshold period of time)
  checkSafetyStop();
  
  // Update the motor power command.  This sets the _currMotorPwr variable.
  updateMotorPwrCmd();
  
  // Update the steering angle if necessary
  if (_prevSteeringAngleCmd != _currSteeringAngleCmd) {
    _prevSteeringAngleCmd = _currSteeringAngleCmd;
    _steeringServo.write(_currSteeringAngleCmd);
  }
  
  // Send a new command to the motor controller if necessary.
  sendMotorPacket();
  
  // Send a status message back to the host computer if necessary.
  sendStatusMsg();
}

// The following encoder interrupts were taken from:
// http://www.arduino.cc/playground/Main/RotaryEncoders

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  _A_set = digitalRead(ENCODER_PIN_A) == HIGH;
  // and adjust counter + if A leads B
  _encoderCnt += (_A_set != _B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  _B_set = digitalRead(ENCODER_PIN_B) == HIGH;
  // and adjust counter + if B follows A
  _encoderCnt += (_A_set == _B_set) ? +1 : -1;
}
