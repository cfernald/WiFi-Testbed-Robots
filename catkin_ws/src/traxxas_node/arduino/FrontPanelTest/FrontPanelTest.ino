/*
 * Blinks the LEDS used by the Proteus III Traxxas controller.
 * This is used to test the front panel display.
 *
 * @author Chien-Liang Fok
 */
enum PIN_ASSIGNMENTS {
  LED_MOTOR_INIT_PIN = 5, // high when the motor controller is initialized
  LED_SAFETY_STOP_PIN = 6,
  LED_CMD_RCVD_PIN = 13, // toggled when ackermann command received
};

int leds[] = {LED_MOTOR_INIT_PIN, LED_SAFETY_STOP_PIN, LED_CMD_RCVD_PIN};
int numLeds = sizeof(leds) / sizeof(int);

void setup() {
  for (int i=0; i < numLeds; i++) {
    pinMode(leds[i], OUTPUT);
  }
}

void loop() {
  for (int i=0; i < numLeds; i++) {
    for (int j=0; j < 6; j++) {
      if (j % 2 == 0)
        digitalWrite(leds[i], HIGH);
      else
        digitalWrite(leds[i], LOW);
      delay(100);
    }
  }
}
