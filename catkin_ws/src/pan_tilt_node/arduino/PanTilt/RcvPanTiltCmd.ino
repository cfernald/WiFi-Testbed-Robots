/**
 * Receives incoming PanTiltCmd messages.
 */
void rcvPanTiltCmd() {
  if (Serial.available() >= sizeof(PanTiltCmd)) {
    
    byte startByte = Serial.read();
    if (startByte == PROTEUS_BEGIN) {
      _panTiltCmdBuff[0] = startByte;
      
      // grab the rest of the bytes
      for (int i=1; i < sizeof(PanTiltCmd); i++) {
        _panTiltCmdBuff[i] = Serial.read();
      }
      
      // compute checksum
      byte checksum = 0;
      for (int i=0; i < sizeof(PanTiltCmd) - 1; i++) {
        checksum ^= _panTiltCmdBuff[i];
      }
      if (checksum == _panTiltCmdBuff[sizeof(PanTiltCmd)-1]) {
        // Checksum passed, accept command
        servoPan.writeMicroseconds(panTiltCmd.pan);
        servoTilt.writeMicroseconds(panTiltCmd.tilt);
        toggleCmdRcvdLED();
      } else {
        // Checksum failed!  TODO: Toggle a status LED
        toggleErrChecksumLED();
      }
      
    } else {
      // first byte not start byte, discard it!  
      toggleErrPktLED();
    }
  }
}

void toggleCmdRcvdLED() {
  digitalWrite(LED_CMD_RCVD_PIN, _ledStateCmdRcvd);
  if (_ledStateCmdRcvd == HIGH)
    _ledStateCmdRcvd = LOW;
  else
    _ledStateCmdRcvd = HIGH;
}

void toggleErrChecksumLED() {
}

void toggleErrPktLED() {
}

