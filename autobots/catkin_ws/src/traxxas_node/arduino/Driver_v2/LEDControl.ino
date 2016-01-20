unsigned long LED_BLINK_PERIOD = 50; // The blink period in milliseconds
unsigned long MIN_INTER_BLINK_TIME = 50; // The min time between blinking an LED

// The yellow LED is used to indicate the reception of an Ackermann command.
boolean yellowLEDActive = false;
unsigned long yellowLEDOnTime;
unsigned long yellowLEDOffTime;

// The red LED is used to indicate an error condition
boolean redLEDActive = false;
unsigned long redLEDOnTime;
unsigned long redLEDOffTime;

// The orage LED is used to indicate the status of the motor controller
boolean orangeLEDActive = false;
unsigned long orangeLEDOnTime;
unsigned long orangeLEDOffTime;

// Begins the process of blinking the yellow LED once.
void blinkYellowLED() {
  if (!yellowLEDActive && calcTimeDiff(yellowLEDOffTime, millis()) > MIN_INTER_BLINK_TIME) {
    yellowLEDActive = true;
    yellowLEDOnTime = millis();
    digitalWrite(PIN_YELLOW_LED, HIGH);
  }
}

// Begins the process of blinking the red LED once.
void blinkRedLED() {
  if (!redLEDActive && calcTimeDiff(redLEDOffTime, millis()) > MIN_INTER_BLINK_TIME) {
    redLEDActive = true;
    redLEDOnTime = millis();
    digitalWrite(PIN_RED_LED, HIGH);
  }
}

// Begins the process of blinking the orange LED once.
void blinkOrangeLED() {
  if (!orangeLEDActive && calcTimeDiff(orangeLEDOffTime, millis()) > MIN_INTER_BLINK_TIME) {
    orangeLEDActive = true;
    orangeLEDOnTime = millis();
    digitalWrite(PIN_ORANGE_LED, HIGH);
  }
}

/**
 * This needs to be called periodically by the main control loop.
 * It refreshes the status of the LEDs.
 */
void updateLEDs() {
  if (yellowLEDActive) {
    if (calcTimeDiff(yellowLEDOnTime, millis()) > LED_BLINK_PERIOD) { // If time to turn off yellow LED
      yellowLEDActive = false;
      yellowLEDOffTime = millis();
      digitalWrite(PIN_YELLOW_LED, LOW);
    }
  }
  
  if (redLEDActive) {
    if (calcTimeDiff(redLEDOnTime, millis()) > LED_BLINK_PERIOD) { // If time to turn off red LED
      redLEDActive = false;
      redLEDOffTime = millis();
      digitalWrite(PIN_RED_LED, LOW);
    }
  }
  
  if (orangeLEDActive) {
    if (calcTimeDiff(orangeLEDOnTime, millis()) > LED_BLINK_PERIOD) { // If time to turn off orange LED
      orangeLEDActive = false;
      orangeLEDOffTime = millis();
      digitalWrite(PIN_ORANGE_LED, LOW);
    }
  }
}
