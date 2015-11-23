/**
 * Tests the speed and steering angle controllers in the Pharos Lab Traxxas Stampede mobility chassis.
 * The speed controller implements a PID loop that adjusts the speed command being
 * sent to the motor controller based on feedback from the encoder.  The user can
 * adjust the target speed by using the '-' and '+' keys through the serial console.
 * The initial target speed is 0m/s.  The target speeds (in cm/s) are defined within the
 * TARGET_SPEED array.
 *
 * The steering angle is controlled by '/' and '*' keys through the serial console.
 *
 * This program runs on an Arduino Pro mini with the encoder channel A attached to pin 2,
 * encoder channel B attached to pin 3, and the motor controller's signal line connected to pin 10.
 *
 * The Syren 25A motor controller must be set on R/C Input mode.  The DIP switches are set as follows:
 * OFF, ON, OFF, ON, ON, ON
 *
 * @author Chien-Liang Fok
 * @date 02/06/2012
 */

// The servo library is used to generate the RC PWM signal that is used by the motor controller.
#include <Servo.h>

int TARGET_SPEED[] = {-200, -175, -150, -125, -100, -75, -50, -25, 0, 25, 50, 75, 100, 125, 150, 175, 200};
#define NUM_SPEEDS 17
int _currentSpeedIndx = 8;

/**
 * Defines the first byte of a message that is sent over the serial port.
 */
//#define PROTEUS_BEGIN 0x24

/*
 * Define the pins used by this program.
 */
enum PIN_ASSIGNMENTS {
  ENCODER_PIN_A = 2,
  ENCODER_PIN_B = 3,
  STEERING_PIN = 9,
  MOTOR_PIN = 10,
  LED_PIN = 13,
};

/*
 * Define the range of parameters for controlling the motor.
 */
#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000
#define MOTOR_MAX_FORWARD 255
#define MOTOR_STOP 84
#define MOTOR_MAX_BACKWARD 0
#define MOTOR_POS_ACCEL_LIMIT 25  // The max positive acceleration in m/s/100ms
#define MOTOR_NEG_ACCEL_LIMIT 50  // The max negative acceleration in m/s/100ms
#define MOTOR_MAX_ERROR 100 // The maximum error in cm/s

/*
 * Define the range of parameters for controlling the steering angle.
 */
#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

byte _ledState = 0;
 
// Variable declarations
Servo _motorControl;

volatile int _encoderCnt = 0;

boolean _A_set = false;
boolean _B_set = false;

/**
 * The time when the speed command was last updated in milliseconds since the MCU was powered on.
 */
unsigned long _prevUpdateTime = 0;

unsigned int _currMotorCmd = MOTOR_STOP; // This is the command send to the Servo component, which generates the RC PWM signal.
int _targetSpeed = 0; // units is cm/s (50cm/s = 0.5m/s)
int _throttledTargetSpeed = 0; // units is cm/s

/**
 * Define the coefficients of the motor PID controller.
 */
#define MOTOR_PID_P 10
#define MOTOR_PID_I 0
#define MOTOR_PID_D 0

int _prevErr = 0;  // The previous error.  This is used when computing the "D" term of the PID controller.
int _totalErr = 0; // The cumulative error since the system started.  This is used by the "I" term in the PID controller

Servo steeringServo;
int _steeringAngle = STEERING_CENTER;

void setup() {
 
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
  
  // Set the target speed.
  _targetSpeed = TARGET_SPEED[_currentSpeedIndx];
  
  steeringServo.attach(STEERING_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  steeringServo.write(STEERING_CENTER);
 
  // Initialize the serial port.  This is for communicating the PID controller's state to an attached PC.
  Serial.begin(115200);
}

void loop() {
  unsigned long currTime = millis();
  
  if (Serial.available()) {
    char dir = Serial.read();
    if (dir == '-') {
      if (_currentSpeedIndx > 0) {
        _currentSpeedIndx--;
        _targetSpeed = TARGET_SPEED[_currentSpeedIndx];
      }
    } else if (dir == '+') {
      if (_currentSpeedIndx < NUM_SPEEDS - 1) {
         _currentSpeedIndx++;
        _targetSpeed = TARGET_SPEED[_currentSpeedIndx];
      }
    } else if (dir == '/') {
      _steeringAngle--;
      if (_steeringAngle < STEERING_MAX_LEFT)
        _steeringAngle = STEERING_MAX_LEFT;
    } else if (dir == '*') {
      _steeringAngle++;
      if (_steeringAngle > STEERING_MAX_RIGHT)
        _steeringAngle = STEERING_MAX_RIGHT;
    }
    
    steeringServo.write(_steeringAngle);
    
    Serial.print("Target speed: ");
    Serial.print(_targetSpeed);
    Serial.print("Steering angle: ");
    Serial.println(_steeringAngle);
    
    toggleLED();
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
    
    Serial.print("target: ");
    Serial.print(_targetSpeed);
    Serial.print(", actual: ");
    Serial.print(currSpeed);
    Serial.print(", motor cmd: ");
    Serial.print(_currMotorCmd);
    Serial.print(", total err: ");
    Serial.print(_totalErr);
    Serial.print(", prev err: ");
    Serial.println(_prevErr);
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
  digitalWrite(LED_PIN, _ledState);
  _ledState != _ledState;
}

