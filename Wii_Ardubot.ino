/*
 (Copy and paste)
 Sparkfun Ardubot + Nunchuck adapter + wireless Nunchuck controller from Logic3
 
 Adapted from :
 http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1259091426
 
 Right motor is controlled by pin 9 and pin 6,
 Left motor is controlled by pin 5 and pin 3
 
 Nunchuck receiver power is connected to Arduino +3.3v and GND
 SDA (data line) is connected to Arduino analog input pin 4,
 SCL (clock line) is connected to Arduino analog input pin 5.
 
 Button "Z" is used to switch between joystick (default) control and accelerometer control
 
 // read out a Wii Nunchuck controller
 // adapted to work with wireless Nunchuck controllers of third party vendors by Michael Dreher <michael@5dot1.de>
 // use "The New Way" of initialization
 <http://wiibrew.org/wiki/Wiimote#The_New_Way>
 
 */

const byte WII_IDENT_LEN = 6;
const byte WII_TELEGRAM_LEN = 6;
const byte WII_NUNCHUCK_TWI_ADR = 0x52;

const byte right1 = 9; // Right motor control 1
const byte right2 = 6; // Right motor control 2

const byte left1 = 5; // Left motor control 1
const byte left2 = 3; // Left motor control 2

//const byte joy_x_mid = 133;    // Sample value
//const byte joy_y_mid = 130;    // Sample value
const byte joy_x_mid = 129;
const byte joy_y_mid = 133;

const int acc_x_mid = 538;
const int acc_y_mid = 540;

int button_state = 1; // variable for reading the Z-button status
int led_state = 0;    // variable containing LED status
int change = 0;
int loop_counter = 0;

#include <Wire.h>

byte outbuf[WII_TELEGRAM_LEN]; // array to store arduino output
int cnt = 0;

void setup()
{

  Wire.begin(); // initialize i2c

  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);

  nunchuck_init(0); // send the initialization handshake
}

// params:
// timeout: abort when timeout (in ms) expires, 0 for unlimited timeout
// return: 0 == ok, 1 == timeout
byte nunchuck_init(unsigned short timeout)
{
  byte rc = 1;
  unsigned long time = millis();
  do
  {
    Wire.beginTransmission(WII_NUNCHUCK_TWI_ADR); // transmit to device 0x52
    Wire.write((byte)0xF0); // sends memory address
    Wire.write((byte)0x55); // sends data.
    if(Wire.endTransmission() == 0) // stop transmitting
    {
      Wire.beginTransmission(WII_NUNCHUCK_TWI_ADR); // transmit to device 0x52
      Wire.write((byte)0xFB); // sends memory address
      Wire.write((byte)0x00); // sends sent a zero.
      if(Wire.endTransmission() == 0) // stop transmitting
      {
        rc = 0;
      }
    }
  }
  while (rc != 0 && (!timeout || ((millis() - time) < timeout)));
  return rc;
}



void clearTwiInputBuffer(void)
{
  // clear the receive buffer from any partial data
  while( Wire.available())
    Wire.read();
}


void send_zero ()
{
  // I don't know why, but it only works correct when doing this exactly 3 times
  // otherwise only each 3rd call reads data from the controller (cnt will be 0 the other times)
  for(byte i = 0; i < 3; i++)
  {
    Wire.beginTransmission (WII_NUNCHUCK_TWI_ADR); // transmit to device 0x52
    Wire.write((byte)0x00); // sends one byte
    Wire.endTransmission(); // stop transmitting
    //delay(1);
  }
}

void loop (){
  delay(100);
  send_zero(); // send the request for next bytes
  Wire.requestFrom(WII_NUNCHUCK_TWI_ADR, WII_TELEGRAM_LEN); // request data from nunchuck

  for (cnt = 0; (cnt < WII_TELEGRAM_LEN) && Wire.available(); cnt++)
  {
    outbuf[cnt] = Wire.read(); // receive byte as an integer
  }

  // debugging

  clearTwiInputBuffer();

  // If we received the 6 bytes, then use them ...
  if (cnt >= WII_TELEGRAM_LEN){
    button_state = bitRead(outbuf[5], 0); // Read input value
    if (button_state == 0) {              // Check if Z-button is  pressed
      // Yes, input is 0
      // If LED is on, turn it off
      // If LED is off, turn it on
      if ((led_state == 0) && (change == 0)){
        digitalWrite(13, HIGH);           // turn LED ON, accel control
        led_state = 1;
        change = 1;                       // Set change, prevent new change
      }
      if ((led_state == 1) && (change == 0)){
        digitalWrite(13, LOW);            // turn LED OFF, joystick control
        led_state = 0;
        change = 1;                       // Set change, prevent new change
      }
    }
    else {                                // Z-button released,
      change = 0;                         // input is HIGH again, reset change,
    }                                     // allow new change

    if(led_state == 0){                   // Joystick control
      byte joy_x = outbuf[0];
      byte joy_y = outbuf[1];

      int xval = joy_x - joy_x_mid;
      int yval = joy_y - joy_y_mid;
      
      if(yval >= 0){
        leftForw(constrain(yval + xval, 0, 255));
        rightForw(constrain(yval - xval, 0, 255));
      }
      if(yval < 0){
        leftBackw(constrain(-yval + xval, 0, 255));
        rightBackw(constrain(-yval - xval, 0, 255));
      }
      if (loop_counter == 20){
        loop_counter = 0;
        Serial.print(joy_x);
        Serial.print(",");
        Serial.print(joy_y);
        Serial.print("\t->\t");
        Serial.print(xval);
        Serial.print(",");
        Serial.print(yval);
        Serial.println("");
      }
    }
  }
  if(led_state == 1){                     // Accelerometer control
    unsigned int accel_x_axis = outbuf[2] << 2;
    unsigned int accel_y_axis = outbuf[3] << 2;

    if (bitRead(outbuf[5], 2) == 1){
      bitSet(accel_x_axis, 1);
    }
    if (bitRead(outbuf[5], 3) == 1){
      bitSet(accel_x_axis, 0);
    }
    if (bitRead(outbuf[5], 4) == 1){
      bitSet(accel_y_axis, 1);
    }
    if (bitRead(outbuf[5], 5) == 1){
      bitSet(accel_y_axis, 0);
    }
    int xval = accel_x_axis - acc_x_mid;
    int yval = accel_y_axis - acc_y_mid;

    if(yval >= 0){
      leftForw(constrain(yval + xval, 0, 255));
      rightForw(constrain(yval - xval, 0, 255));
    }
    if(yval < 0){
      leftBackw(constrain(-yval + xval, 0, 255));
      rightBackw(constrain(-yval - xval, 0, 255));
    }
    if (loop_counter == 20){
      loop_counter = 0;
      Serial.print(accel_x_axis);
      Serial.print(",");
      Serial.print(accel_y_axis);
      Serial.print("\t->\t");
      Serial.print(xval);
      Serial.print(",");
      Serial.print(yval);
      Serial.println("");      
    }
  }
  loop_counter++;
}

void rightForw(byte speed){              // Motor code blocks
  digitalWrite(right1, LOW);
  analogWrite(right2, speed);
}
void rightBackw(byte speed){
  analogWrite(right1, speed);
  digitalWrite(right2, LOW);
}

void leftForw(byte speed){
  analogWrite(left1, LOW);
  digitalWrite(left2, speed);
}
void leftBackw(byte speed){
  digitalWrite(left1, speed);
  analogWrite(left2, LOW);
}

