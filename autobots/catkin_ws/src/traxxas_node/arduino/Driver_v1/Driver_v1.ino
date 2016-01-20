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
 * @author Chien-Liang Fok
 * @date 02/08/2012
 */

#include <Servo.h> // Used to generate the PWM signal to the steering servo and motor controller

#define PROTEUS_BEGIN 0x24

/*
 * Define the range of parameters for controlling the motor.
 */
#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000
#define MOTOR_MAX_FORWARD 255
#define MOTOR_STOP 85
#define MOTOR_MAX_BACKWARD 0
#define MOTOR_POS_ACCEL_LIMIT 25  // The max positive acceleration in m/s/100ms
#define MOTOR_NEG_ACCEL_LIMIT 50  // The max negative acceleration in m/s/100ms
#define MOTOR_MAX_ERROR 100 // The maximum error in cm/s

/**
 * Define the coefficients of the motor PID controller.
 */
#define MOTOR_PID_P 10
#define MOTOR_PID_I 0
#define MOTOR_PID_D 0

/*
 * Define the range of parameters for controlling the steering angle.
 */
#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

/**
 * This is the number of milliseconds that can pass in which no
 * move command is received before the motor is stopped.
 */
#define SAFETY_STOP_INTERVAL 500

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
  uint16_t motorCmd;
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
  ENCODER_PIN_A = 2,
  ENCODER_PIN_B = 3,
  STEERING_PWM_PIN = 9,
  MOTOR_PIN = 10,
  LED_PIN = 13,
};


byte _ledState = HIGH;
byte* _moveCmdBuff = (byte*)&moveCmd;
byte* _statusMsgBuff = (byte*)&statusMsg;

// Variable declarations
Servo _motorControl;

volatile int _encoderCnt = 0;

boolean _A_set = false;
boolean _B_set = false;

/**
 * Records when the speed command was last updated by the PID controller.
 * Unit is in milliseconds since power on.
 */
unsigned long _prevUpdateTime = 0;

/**
 * Records when the most recent move command was received from the x86.
 * Unit is in milliseconds since power on.
 */
unsigned long _prevCmdTime = 0;

unsigned int _currMotorCmd = MOTOR_STOP; // This is the command send to the Servo component, which generates the RC PWM signal.
int _targetSpeed = 0; // units is cm/s (50cm/s = 0.5m/s)
int _throttledTargetSpeed = 0; // units is cm/s

int _prevErr = 0;  // The previous error.  This is used when computing the "D" term of the PID controller.
int _totalErr = 0; // The cumulative error since the system started.  This is used by the "I" term in the PID controller

Servo steeringServo;
int _targetSteeringAngle; // 1/10 degree
int _currSteeringAngle; // 1/10 degree
int _currSteeringAngleCmd = STEERING_CENTER; // servo units
// TODO: Add a throttled steering angle that changes the steering angle based on a set rate

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Initialize LED
 
  // Configure the encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  digitalWrite(ENCODER_PIN_A, HIGH);  // turn on pullup resistor
  digitalWrite(ENCODER_PIN_B, HIGH);  // turn on pullup resistor

  // Configure the encoder interrupts
  attachInterrupt(0, doEncoderA, CHANGE); // encoder channel A is on pin 2, interrupt 0
  attachInterrupt(1, doEncoderB, CHANGE); // encoder channel B is on pin 3, interrupt 1
 
  // Initialize the motor controller.  The motor is controlled via RC PWM signal
  _motorControl.attach(MOTOR_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
  _motorControl.write(MOTOR_STOP);
  
  steeringServo.attach(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  steeringServo.write(STEERING_CENTER);
 
  // Initialize the serial port.  This is for communicating the PID controller's state to an attached PC.
  Serial.begin(115200);
}

void loop() {
  unsigned long currTime = millis();
  
  // Read at most one command
  if (Serial.available() >= sizeof(MoveCmd)) {
    
    byte startByte = Serial.read();
    if (startByte == PROTEUS_BEGIN) {
      _moveCmdBuff[0] = startByte;
      
      // grab the rest of the bytes
      for (int i=1; i < sizeof(MoveCmd); i++) {
        _moveCmdBuff[i] = Serial.read();
      }
      
      // compute checksum
      byte checksum = 0;
      for (int i=0; i < sizeof(MoveCmd) - 1; i++) {
        checksum ^= _moveCmdBuff[i];
      }
      if (checksum == _moveCmdBuff[sizeof(moveCmd)-1]) {
        // Checksum passed, accept command
        _currSteeringAngle = _targetSteeringAngle = moveCmd.steering; 
        
        _currSteeringAngleCmd = 100 - _targetSteeringAngle * 8 / 20; // convert from 1/10 degree into servo command unit.  Valid range -200 to 200.
        _targetSpeed = moveCmd.speed;
        
        _prevCmdTime = currTime;
        
      } else {
        // Checksum failed!  TODO: Toggle a status LED
        
      }
      
    } else {
      // first byte not start byte, discard it!  
      toggleLED();
    }
    
    steeringServo.write(_currSteeringAngleCmd);
  }
  
  /**
   * Check for safety stop condition.  This is triggered when no
   * data is received within the SAFETY_STOP_INTERVAL.
   */
   if (calcTimeDiff(_prevCmdTime, currTime) > SAFETY_STOP_INTERVAL) {
     // TODO: Toggle LED indicating error condition
     _targetSpeed = 0;  // set speed to be 0 cm/s
     steeringServo.write(STEERING_CENTER);
   }
  
  /**
   * Update the throttled target speed and PID controller every 100ms.
   */
  if (calcTimeDiff(_prevUpdateTime, currTime) >= 100) {
    int encoderCnt = _encoderCnt;
    _encoderCnt = 0;
    _prevUpdateTime = currTime;
    
    // Compute the number of encoder counts per second.
    // Since encoderCnt is the number of counter per 100ms, multiply by 10 to get
    // number of counts per second.
    int cntPerSecond = encoderCnt * 10;
   
    // Calculate speed in cm/s.  To do this, divide cntPerSecond
    // by 1000 (since there are 1000 encoder counts per wheel rotation), then multiple
    // by 36 (since the wheel's circumference is 36cm).  Collectivity,
    // this is approximately equal to dividing cntPerSecond by 28.
    int currSpeed = cntPerSecond / 28; // / 1000 * 36; 
    
    // Update the throttled target speed.  This implements software-based acceleration/deceleration.
    if (_throttledTargetSpeed != _targetSpeed) {
      if (_throttledTargetSpeed < _targetSpeed) {
        // Must increase speed
        if (MOTOR_POS_ACCEL_LIMIT == 0 || _targetSpeed - _throttledTargetSpeed < MOTOR_POS_ACCEL_LIMIT)
          _throttledTargetSpeed = _targetSpeed;
        else
          _throttledTargetSpeed += MOTOR_POS_ACCEL_LIMIT;
      } else {
        // Must decrease speed
        if (MOTOR_NEG_ACCEL_LIMIT == 0 || _throttledTargetSpeed - _targetSpeed < MOTOR_NEG_ACCEL_LIMIT)
          _throttledTargetSpeed = _targetSpeed;
        else
          _throttledTargetSpeed -= MOTOR_NEG_ACCEL_LIMIT;
      }
    }
    
    // Check for special stop condition
    if (_throttledTargetSpeed == 0) {
      _currMotorCmd = MOTOR_STOP;
      _prevErr = _totalErr = 0;
    } else {
      // Calculate the speed error in cm/s.  A positive value means the robot
      // is currently traveling too slow and must speed up.
      int currSpeedErr = _throttledTargetSpeed - currSpeed;
   
      // Update the PID controller terms.
      _totalErr += currSpeedErr;
      int deltaErr = currSpeedErr - _prevErr;
      _prevErr = currSpeedErr;
    
      if (abs(_prevErr) > MOTOR_MAX_ERROR) {
        _currMotorCmd = MOTOR_STOP;
        _prevErr = _totalErr = _throttledTargetSpeed = 0;
      } else {
        // Generate the new motor command
        _currMotorCmd += currSpeedErr/MOTOR_PID_P + MOTOR_PID_I * _totalErr + MOTOR_PID_D * deltaErr;
      }
      
      // Apply the cutoffs to ensure the robot does not move too fast
      if (_currMotorCmd > MOTOR_MAX_FORWARD) {
        _currMotorCmd = MOTOR_MAX_FORWARD;
      } else if (_currMotorCmd < MOTOR_MAX_BACKWARD) {
        _currMotorCmd = MOTOR_MAX_BACKWARD;
      }
    }
    
    // Send the new motor command to the motor.
    _motorControl.write(_currMotorCmd);
    
    statusMsg.begin = PROTEUS_BEGIN;
    statusMsg.targetSpeed = _targetSpeed;
    statusMsg.currSpeed = currSpeed;
    statusMsg.motorCmd = _currMotorCmd;
    statusMsg.prevErr = _prevErr;
    statusMsg.totalErr = _totalErr;
    statusMsg.targetSteeringAngle = _targetSteeringAngle;
    statusMsg.currSteeringAngle = _currSteeringAngle;
    statusMsg.steeringAngleCmd = _currSteeringAngleCmd;
    
    // compute checksum
    byte checksum = 0;
    for (int i=0; i < sizeof(StatusMsg) - 1; i++) {
      checksum ^= _statusMsgBuff[i];
    }
    statusMsg.checksum = checksum;
    
    Serial.write((byte*)&statusMsg, sizeof(statusMsg));
    Serial.flush();
    /*Serial.print("target: ");
    Serial.print(_targetSpeed);
    Serial.print(", actual: ");
    Serial.print(currSpeed);
    Serial.print(", motor cmd: ");
    Serial.print(_currMotorCmd);
    Serial.print(", total err: ");
    Serial.print(_totalErr);
    Serial.print(", prev err: ");
    Serial.println(_prevErr);*/
  }
}

unsigned long calcTimeDiff(unsigned long time1, unsigned long time2) {
  if (time1 > time2) {
    unsigned long maxUL = 0xFFFFFFFF;
    return maxUL - time2 + time1;
  } else
    return time2 - time1;
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

void toggleLED() {
  digitalWrite(13, _ledState);
  if (_ledState == HIGH)
    _ledState = LOW;
  else
    _ledState = HIGH;
}
