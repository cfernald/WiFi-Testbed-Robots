/**
 * Tests the encoder on the Pharos Lab Traxxas Stampede mobility chassis.
 * It slowly spins the wheels while monitoring the encoder and sends the encoder
 * count over the serial port for display on an attached PC.  Thus, be sure the rear
 * wheels are elevated when running this program!
 *
 * It allows you to visually monitor the number of times the wheel has rotated,
 * correlate it to the encoder count, and determine the number of encoder counts
 * per rotation.
 * 
 * Using this program, it was determined that each wheel rotation generates 1000 encoder counts.
 *
 * This program runs on an Arduino Pro mini with the encoder channel A attached to pin 2,
 * encoder channel B attached to pin 3, and the motor controller's signal line connected to pin 10.
 *
 * @author Chien-Liang Fok
 * @date 02/06/2012
 */
#include <Servo.h>

/*
 * Define the pins used by this program.
 */
enum PIN_ASSIGNMENTS {
  ENCODER_PIN_A = 2,
  ENCODER_PIN_B = 3,
  STEERING_PIN = 9,
  MOTOR_PIN = 10,
};

/*
 * Define the range of parameters for controlling the motor.
 */
#define MOTOR_MIN_PULSE 1000
#define MOTOR_MAX_PULSE 2000
#define MOTOR_MAX_FORWARD 255
#define MOTOR_STOP 85
#define MOTOR_MAX_BACKWARD 0
#define MOTOR_SPEED_CMD 88 // move forward slowly (88 is the slowest forward movement,  is the slowest rearward movement)

#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000
#define STEERING_CENTER 100

// Variable declarations
Servo _motorControl;
Servo _steeringServo;

volatile unsigned int _encoderPos = 0;
unsigned int _lastReportedPos = 1;

boolean _A_set = false;
boolean _B_set = false;


void setup() { 
  pinMode(ENCODER_PIN_A, INPUT); 
  pinMode(ENCODER_PIN_B, INPUT); 
  digitalWrite(ENCODER_PIN_A, HIGH);  // turn on pullup resistor
  digitalWrite(ENCODER_PIN_B, HIGH);  // turn on pullup resistor

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  _motorControl.attach(MOTOR_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
  _motorControl.write(MOTOR_SPEED_CMD); 
  
  _steeringServo.attach(STEERING_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  _steeringServo.write(STEERING_CENTER);
  
  Serial.begin(115200);
}

void loop() { 
  if (_lastReportedPos != _encoderPos) {
    Serial.print("Index:");
    Serial.print(_encoderPos, DEC);
    Serial.println();
    _lastReportedPos = _encoderPos;
  }
}

// The following encoder interrupts were taken from:
// http://www.arduino.cc/playground/Main/RotaryEncoders

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  _A_set = digitalRead(ENCODER_PIN_A) == HIGH;
  // and adjust counter + if A leads B
  _encoderPos += (_A_set != _B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  _B_set = digitalRead(ENCODER_PIN_B) == HIGH;
  // and adjust counter + if B follows A
  _encoderPos += (_A_set == _B_set) ? +1 : -1;
}
