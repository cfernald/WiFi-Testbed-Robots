
/**
 * This is the amount of time in milliseconds to wait after the 
 * motor controller pulls the MOTOR_STATUS_PIN high before 
 * sending a start byte to the motor controller.
 */
#define MOTOR_BOOT_INTERVAL 3000

/**
 * The threshold interval in milliseconds between movement commands
 * before the motor is stopped.  The ROS driver is programed to send
 * a command message every 200ms.  Thus the safety interval is set 
 * to 300ms.
 */
#define SAFETY_STOP_INTERVAL 300

/**
 * The max positive acceleration in cm/s/100ms.  A value of 2 means that
 * it will take the robot (0.5 m/s * 100) / 2 * 100 / 1000 = 2.5s to go from 0m/s to 0.5m/s.
 */
#define MOTOR_POS_ACCEL_LIMIT 2

/**
 * The max negative acceleration in cm/s/100ms.
 */
#define MOTOR_NEG_ACCEL_LIMIT 4

/**
 * The maximum error in cm/s
 */
#define MOTOR_MAX_ERROR 30

/**
 * Define the coefficients of the motor PID controller.
 */
#define MOTOR_PID_P 10
#define MOTOR_PID_I 0
#define MOTOR_PID_D 0

byte MOTOR_START_BYTE = 170;
byte MOTOR_ADDR = 128;
byte MOTOR_CMD_FORWARD = 0;
byte MOTOR_CMD_BACKWARD = 1;
byte MOTOR_PWR_MAX = 127;
byte MOTOR_PWR_STOP = 0;

/**
 * Whether an init byte was sent to the motor controller.
 */
boolean _motorInit = false;

/**
 * Whether the motor is on.
 */
boolean _motorOn = false;

/**
 * Records when the motor controller was powered on.
 */
unsigned long _motorOnTime = 0;

/**
 * Records when the speed command was last updated by the PID controller.
 */
unsigned long _prevPIDRun = 0;

/**
 * The current motor power.  This is set by the PID controller, range -MOTOR_PWR_MAX to MOTOR_PWR_MAX
 */
int _currMotorPwr = MOTOR_PWR_STOP;
int _prevMotorPwr = 0xffff;

/**
 * Checks whether MOTOR_START_BYTE should be sent to the motor controller.
 * An init byte should be sent if the motor controller just turned on and
 * MOTOR_BOOT_INTERVAL has passed.
 * 
 * This method should be repeatedly called by the main loop within Driver_v2
 */
void checkMotorControllerStatus() {
  int motorIsOn = digitalRead(MOTOR_STATUS_PIN);
  
  // If the motor controller is not on (as indicated by the fact that it has
  // not pulled MOTOR_STATUS_PIN high), record this fact in _motorOn and turn
  // off the MOTOR_INIT (orange) LED on the front panel.
  if (!motorIsOn) {
    _motorOn = false;
    _motorInit = false;
    blinkOrangeLED();

//    Serial.print("Motor is not ON");  //PEDRO-DEBUG
  }
  
  // If the motor controller has just turned on, wait MOTOR_BOOT_INTERVAL milliseconds
  // and then send it a MOTOR_START_BYTE.  This sets the baud rate used by the motor 
  // controller.
  else {
    if (!_motorOn) {
      _motorOn = true;
      _motorOnTime = millis();
    } else {
      if (!_motorInit && calcTimeDiff(_motorOnTime, millis()) > MOTOR_BOOT_INTERVAL) {
        _motorPort.write(MOTOR_START_BYTE); // set the baud rate
        _motorInit = true;
        digitalWrite(PIN_ORANGE_LED, HIGH);
      }
    }
  }
}

/**
 * Sends a packet to the motor controller telling it how fast to move.
 * It relies on the globally-defined variable "_currMotorPwr".
 */
void sendMotorPacket() {
  if (_servoDone && _prevMotorPwr != _currMotorPwr) {
    _servoDone = false;
    byte currMotorAbsPwr = abs(_currMotorPwr);
  
    byte currMotorCmd = MOTOR_CMD_FORWARD;
    if (_currMotorPwr < 0) {
      currMotorCmd = MOTOR_CMD_BACKWARD;
    }
  
    byte motorChecksum = 0x7F & (MOTOR_ADDR + currMotorCmd + currMotorAbsPwr);
  
    if (_motorInit && !_steeringServo.isBusy()) {  
      _motorPort.write(MOTOR_ADDR); // send address byte
      _motorPort.write(currMotorCmd);
      _motorPort.write(currMotorAbsPwr);
      _motorPort.write(motorChecksum);
      _prevMotorPwr = _currMotorPwr;
    }
  }
}

/**
 * Check for safety stop condition.  This is triggered when no
 * data is received within the SAFETY_STOP_INTERVAL.
 */
void checkSafetyStop() {
   if (calcTimeDiff(_prevCmdTime, millis()) > SAFETY_STOP_INTERVAL) {
     blinkRedLED();
     _targetSpeed = 0;  // set speed to be 0 cm/s
     _currSteeringAngleCmd = STEERING_CENTER; // center the front wheels
   }
}

/**
 * Compute the new motor power based on the target and measured
 * speeds.  Calling this method sets the _currMotorPwr variable.
 * This method can be called as frequently as possible but will only execute
 * at 10Hz.
 */
void updateMotorPwrCmd() {
  /**
   * Update the throttled target speed and PID controller every 100ms.
   */
  if (calcTimeDiff(_prevPIDRun, millis()) >= 100) {
    int encoderCnt = _encoderCnt; // grab the current encoder count
    _encoderCnt = 0;
    
    _prevPIDRun = millis(); // record when this controller was last run
    
    // Compute the encoder counts per second.
    // Since encoderCnt is the number of counts per 100ms, multiply by 10 to get
    // number of counts per second.
    int cntPerSecond = encoderCnt * 10;
   
    // Calculate speed in cm/s.  To do this, divide cntPerSecond
    // by 1000 (since there are 1000 encoder counts per wheel rotation), then multiple
    // by 36 (since the wheel's circumference is 36cm).  Collectivity,
    // this is approximately equal to dividing cntPerSecond by 28.
    _currSpeed = cntPerSecond / 28; // / 1000 * 36; 
    
    // Update the throttled target speed.  This implements software-based acceleration/deceleration.
    if (_throttledTargetSpeed != _targetSpeed) {
      if (_throttledTargetSpeed > 0 ) {
      // The robot is currently moving forward
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
      } else {
        // The robot is currently moving backward
        if (_throttledTargetSpeed < _targetSpeed) {
          // Must decrease speed
          if (MOTOR_NEG_ACCEL_LIMIT == 0 || _targetSpeed - _throttledTargetSpeed < MOTOR_NEG_ACCEL_LIMIT)
            _throttledTargetSpeed = _targetSpeed;
          else
            _throttledTargetSpeed += MOTOR_NEG_ACCEL_LIMIT;
        } else {
          // Must increase speed
          if (MOTOR_POS_ACCEL_LIMIT == 0 || _throttledTargetSpeed - _targetSpeed < MOTOR_POS_ACCEL_LIMIT)
            _throttledTargetSpeed = _targetSpeed;
          else
            _throttledTargetSpeed -= MOTOR_POS_ACCEL_LIMIT;
        }
      }
    }
    
    // Check for special stop condition
    if (_throttledTargetSpeed == 0) {
      _currMotorPwr = MOTOR_PWR_STOP;
      _prevErr = _totalErr = 0;
    } else {
      // Calculate the speed error in cm/s.  A positive value means the robot
      // is currently traveling too slow and must speed up.
      int currSpeedErr = _throttledTargetSpeed - _currSpeed;
   
      // Update the PID controller terms.
      _totalErr += currSpeedErr;
      int deltaErr = currSpeedErr - _prevErr;
      _prevErr = currSpeedErr;
    
      if (abs(_prevErr) > MOTOR_MAX_ERROR) {
        _currMotorPwr = MOTOR_PWR_STOP;
        _prevErr = _totalErr = _throttledTargetSpeed = 0;
      } else {
        // Generate the new motor command
        _currMotorPwr += currSpeedErr/MOTOR_PID_P + MOTOR_PID_I * _totalErr + MOTOR_PID_D * deltaErr;
      }
      
      // Apply the cutoffs to ensure the robot does not move too fast
      if (_currMotorPwr > MOTOR_PWR_MAX) {
        _currMotorPwr = MOTOR_PWR_MAX;
      }
    }
  }
}
