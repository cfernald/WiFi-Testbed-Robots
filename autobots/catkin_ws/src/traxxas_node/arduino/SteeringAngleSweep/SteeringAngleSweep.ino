/**
 * Sweeps the Traxxas Stampede's steering angle across its entire range.
 * Pauses for half a second at each limit and in the center.
 *
 * @author Chien-Liang Fok
 * @date 02/06/2012
 */
#include <Servo.h>

#define STEERING_PWM_PIN 9
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000

#define PAUSE 10 // in milliseconds

Servo steeringServo;
int pos;

void setup() {
  steeringServo.attach(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  steeringServo.write(STEERING_CENTER);
  delay(2000);
}

void loop() {
  
  for(pos = STEERING_CENTER; pos < STEERING_MAX_RIGHT; pos++) {
    steeringServo.write(pos);
    delay(PAUSE);
  }
  
  delay(500);
  
  for(pos = STEERING_MAX_RIGHT; pos >= STEERING_CENTER; pos--) {                                
    steeringServo.write(pos);
    delay(PAUSE);
  }
  
  delay(500);
  
  for(pos = STEERING_CENTER; pos >= STEERING_MAX_LEFT; pos--) {                                
    steeringServo.write(pos);
    delay(PAUSE);
  }
  
  delay(500);
  
  for(pos = STEERING_MAX_LEFT; pos <= STEERING_CENTER; pos++) {                                
    steeringServo.write(pos);
    delay(PAUSE);
  }
  
  delay(500);
  
}
