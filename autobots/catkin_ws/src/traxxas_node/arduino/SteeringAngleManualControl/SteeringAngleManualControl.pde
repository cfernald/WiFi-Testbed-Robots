/**
 * Enables manual control of the Traxxas Stampede's steering angle.  
 * After programing the Arduino with this sketch, open a serial console and
 * set the line ending to be "Newline".  Then type the desired steering angle
 * in tenths of a degree followed by [enter].  The valid range is from -200 (full-lock right) to 
 * 200 (full-lock left) degrees, with 0 being center.
 *
 * @author Chien-Liang Fok
 * @date 02/08/2012
 */
#include <Servo.h>

#define STEERING_PWM_PIN 9
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000

#define LED_PIN 13  // This is connected to the on-board LED

Servo steeringServo;
byte _ledState = 0;
char _buff[10];
int _indx = 0;
  
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  // Initialize LED
  steeringServo.attach(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  steeringServo.write(STEERING_CENTER);
  delay(2000);
}

void loop() {
  while (Serial.available() && _indx < 9) {
    _buff[_indx++] = Serial.read();
    if (_buff[_indx-1] == 0x0A) {  // if last char is newline char
      _indx = 0;
      int steeringAngle = atoi(_buff);
      int steeringAngleCmd = 100 - steeringAngle * 8 / 20; // steeringAngle should be in tenth of a degree
    
      if (steeringAngleCmd < STEERING_MAX_LEFT)
        steeringAngleCmd = STEERING_MAX_LEFT;
   
      if (steeringAngleCmd > STEERING_MAX_RIGHT)
        steeringAngleCmd = STEERING_MAX_RIGHT;
        
      Serial.print("Steering angle: ");
      Serial.print(steeringAngle);
      Serial.print(", Cmd: ");
      Serial.println(steeringAngleCmd);
  
      steeringServo.write(steeringAngleCmd);
    
      digitalWrite(LED_PIN, _ledState);
      _ledState = !_ledState; 
    }
  }
}
