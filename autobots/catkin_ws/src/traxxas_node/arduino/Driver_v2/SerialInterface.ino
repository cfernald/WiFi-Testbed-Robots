/**
 * The serial interface contains methods that interact with the device (usually an x86) attached
 * to the serial port of the Arduino.
 */


/**
 * The interval in milliseconds between sending status messages to the attached computer.
 */
unsigned long TIME_INTERVAL_SEND_STATUS = 200; // 5 Hz

/**
 * This is the time when a status message was last sent to the attached computer
 */
unsigned long _prevStatusSndTime;

/**
 * Receives incoming AckermannCmd messages.  It sets global variables:
 *   - _currSteeringAngle
 *   - _currSteeringAngleCmd 
 *   - _targetSpeed.
 */
void rcvAckermannCmd() {
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
        
        _prevCmdTime = millis();
        
        blinkYellowLED();
      } else {
        // Checksum failed!
        blinkRedLED();
        // TODO: Send error status message to x86
      }
      
    } else {
      // first byte not start byte, discard it!  
      blinkRedLED();
    }
  }
}

/**
 * Sends a status message to the x86.
 */
void sendStatusMsg() {
  if (calcTimeDiff(_prevStatusSndTime, millis()) > TIME_INTERVAL_SEND_STATUS) {
    _prevStatusSndTime = millis();
    statusMsg.begin = PROTEUS_BEGIN;
    statusMsg.targetSpeed = _targetSpeed;
    statusMsg.currSpeed = _currSpeed;
    statusMsg.motorPwr = _currMotorPwr;
    statusMsg.prevErr = _prevErr;
    statusMsg.totalErr = _totalErr;
    statusMsg.targetSteeringAngle = _targetSteeringAngle;
    statusMsg.currSteeringAngle = _currSteeringAngle;
    statusMsg.steeringAngleCmd = _currSteeringAngleCmd;
    
    // compute checksum for message
    byte checksum = 0;
    for (int i=0; i < sizeof(StatusMsg) - 1; i++) {
      checksum ^= _statusMsgBuff[i];
    }
    statusMsg.checksum = checksum;
    
    Serial.write((byte*)&statusMsg, sizeof(statusMsg));
    Serial.flush();
  }
}


