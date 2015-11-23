/**
 * Accesses the HMC6343 compass for the heading, pitch, and roll at 10Hz.
 * Packages the data in a Proteus message and prints it to the serial port.
 *
 * Two LEDs are used:
 *  - pin 6, green, on upon initialization
 *  - pin 7, red, blinked each time the compass is accessed.
 *
 * The message transmitted:
 *  [proteus begin (1 byte)][heading (2 bytes)][pitch (2 bytes)][roll (2 bytes)][checksum (1 byte)]
 *
 * The checksum is the XOR of the other bytes in the message.
 *
 * @author Chien-Liang Fok
 */
#include <Wire.h>

/**
 * Define the constants.
 */
#define HMC6343_ADDRESS            0x19
#define HMC6343_HEADING_REG        0x50
#define PROTEUS_BEGIN              0x24

int GREEN_LED = 6;
int RED_LED = 7;

byte buf[8]; // A buffer for holding the message

void setup() {
  buf[0] = PROTEUS_BEGIN;
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  Wire.begin(); // Initialize the I2C bus
  
  digitalWrite(GREEN_LED, HIGH); // Turn on the green LED to incate system started
  Wire.beginTransmission(HMC6343_ADDRESS);    // Start communicating with the HMC6343 compasss in writing mode
  Wire.write(0xF1);             // Crommand to write to an EEPROM Register
  Wire.write(0x04);             // Register at address 0x04 (OP MODE register 1)
  Wire.write(0x31);             // Content to write to the register
  Wire.endTransmission();
  
  Wire.beginTransmission(HMC6343_ADDRESS);    // Start communicating with the HMC6343 compasss in writing mode
  Wire.write(0xF1);             // Crommand to write to an EEPROM Register
  Wire.write(0x14);             // Register at address 0x04 (OP MODE register 1)
  Wire.write((uint8_t) 0x09);             // Content to write to the register
  Wire.endTransmission();
  
  Serial.begin(115200); // Initialize the serial bus
  digitalWrite(GREEN_LED, HIGH); // Turn on the green LED to incate system started
}

void loop() {
  Wire.beginTransmission(HMC6343_ADDRESS);    // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG);             // Send the address of the register that we want to read
  Wire.endTransmission();

  Wire.requestFrom(HMC6343_ADDRESS, 6);    // Request six bytes of data from the HMC6343 compasss
  while(Wire.available() < 1);             // Busy wait while there is no byte to receive
  
  buf[1] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buf[2] = Wire.read();
  //float heading = ((highByte << 8) + lowByte) / 10.0; // the heading in degrees

  buf[3] = Wire.read();
  buf[4] = Wire.read();
  //float pitch = ((highByte << 8) + lowByte) / 10.0;   // the pitch in degrees
  
  buf[5] = Wire.read();
  buf[6] = Wire.read();
  //float roll = ((highByte << 8) + lowByte) / 10.0;    // the roll in degrees
  
  // compute the checksum
  buf[7] = 0;
  for (int i = 0; i < 7; i++) {
    buf[7] ^= buf[i];
  }
  
  Serial.write(buf, 8);
  /*Serial.print("Heading=");             // Print the sensor readings to the serial port.
  Serial.print(heading);
  Serial.print(", Pitch=");
  Serial.print(pitch);
  Serial.print(", Roll=");
  Serial.println(roll);*/
  
  digitalWrite(RED_LED, HIGH);
  delay(20);
  digitalWrite(RED_LED, LOW);
  delay(80);
  
  //delay(100); // Do this at approx 10Hz
}
