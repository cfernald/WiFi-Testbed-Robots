
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
#define COMPASS3                    0x19 //this is the 32 compass
#define COMPASS2                    0x10 // this is the 20 compass
#define COMPASS1                    0x08//this is the 10 compass

#define HMC6343_HEADING_REG         0x50
#define PROTEUS_BEGIN               0x24

int GREEN_LED = 6;
int RED_LED = 7;

byte buf[8]; // A buffer for holding the message

//this is the sending buffer
byte buffer[20];

void setup() {
  buf[0] = PROTEUS_BEGIN;
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  Wire.begin(); // Initialize the I2C bus
  Serial.begin(115200); // Initialize the serial bus
  digitalWrite(GREEN_LED, HIGH); // Turn on the green LED to incate system started
}

void loop() {
  // compute the checksum
  compass1();
  compass2();
  compass3();
  buffer[7] = 0;
  for (int i = 0; i < 19; i++) {
    buffer[19] ^= buffer[i];
  }
  Serial.write(buf, 20);
  digitalWrite(RED_LED, HIGH);
  delay(20);
  digitalWrite(RED_LED, LOW);
  delay(80);

}
//this compass has addresss 32
void compass1(void)
{
  Wire.beginTransmission(COMPASS1);    // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG);             // Send the address of the register that we want to read
  Wire.endTransmission();

  Wire.requestFrom(COMPASS1, 6);    // Request six bytes of data from the HMC6343 compasss
  while(Wire.available() < 1);             // Busy wait while there is no byte to receive
  buffer[1] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[2] = Wire.read();
  buffer[3] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[4] = Wire.read();
  buffer[5] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[6] = Wire.read();
}
//this compas ahs address 20
void compass2(void)
{
  Wire.beginTransmission(COMPASS2);    // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG);             // Send the address of the register that we want to read
  Wire.endTransmission();

  Wire.requestFrom(COMPASS2, 6);    // Request six bytes of data from the HMC6343 compasss
  while(Wire.available() < 1);             // Busy wait while there is no byte to receive
  buffer[7] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[8] = Wire.read();
  buffer[9] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[10] = Wire.read();
  buffer[11] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[12] = Wire.read();
}
//this compass has abn addres of 10
void compass3(void)
{
  Wire.beginTransmission(COMPASS3);    // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG);             // Send the address of the register that we want to read
  Wire.endTransmission();

  Wire.requestFrom(COMPASS3, 6);    // Request six bytes of data from the HMC6343 compasss
  while(Wire.available() < 1);             // Busy wait while there is no byte to receive
  buffer[13] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[14] = Wire.read();
  buffer[15] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[16] = Wire.read();
  buffer[17] = Wire.read();              // Reads in the bytes and convert them into proper degree units.
  buffer[18] = Wire.read();
}
