/*
/---------------------------------------------------------------/
 * Code for the Aus3D Magnetic Encoder I2C Module
 * Chris Barr, 2016
 * 
 * This module is designed to connect to a host device via I2C,
 * and report the current position as observed by the magnetic encoder.
 * 
 * This code is compatible with both AMS AS5311 and NSE-5310 magnetic encoder ICs. The relevant IC must be defined below.
 * 
 * The following I2C commands are implemented:
 *    
 * 1: Resets travelled distance to zero
 * 
 * 2: Sets this modules I2C address to the next byte received, 
 *    saves in EEPROM
 *    Module resets and rejoins bus with new address
 *    
 * 3: Set the information that will be sent on the next I2C request.
 *      0: Default, responds with encoder count
 *      1: Responds with magnetic signal strength
 *            0: Signal good, in range.
 *            1: Signal weak, edge of range (but probably usable)
 *            2: Signal weak / lost, edge / outside of range (probably not usable)
 *      2: Responds with firmware version + compile date / time
 *      3: Responds with raw encoder reading
 *
 * 4: Clear the settings saved in EEPROM, restoring all defaults
 *      Handy to return to hardware I2C address if required
 *      A power cycle will be required for settings to reset
 *
 * 10: Sets the mode of the LEDs based on the next two bytes received, 
 *     byte1 = led (0 or 1), byte 2 = mode (see below)
 *     saves in EEPROM
 *
 * 11: Sets the brightness of the LEDs based on the next two bytes received,
 *    byte1 = led (0 or 1), byte 2 = brightness (0-255)
 *    saves in EEPROM
 *
 * 12: Sets the RGB value for the LEDs to the next three bytes received,
 *     LEDs must still be set to RGB mode to display the sent value.
 *
 * 13: Sets the HSV value for the LEDs to the next three bytes received,
 *     LEDs must still be set to HSV mode to display the sent value.
 *
 * 14: Set the rate variable of the LEDs based on the next two bytes received,
 *      Rate is used in some of the different LED modes
 * 
 * I2C address can either be set over I2C (as shown above), or configured by cutting jumper traces on PCB.
 * If the address has been set by I2C, it can only be overridden by setting a new address over I2C, or by
 * sending the I2C command to reset the EEPROM.
 * 
 * 0 = default
 * 1 = trace cut
 * 
 * ADR1 ADR2 |  Mode
 *  0    0   |   X      //Default
 *  0    1   |   Y
 *  1    0   |   Z
 *  1    1   |   E
 *  
 *  LED Modes:
 *  
 *  0     Status indication of magnetic field
 *  1     Solid White
 *  2     Solid Red
 *  3     Solid Green
 *  4     Solid Blue
 *  5     RGB Value
 *  6     HSV Value
 *  7     Party Mode 1
 *  8     Party Mode 2
 *              
 *              
 *  Requires Arduino on Breadboard hardware core from this page:
 *  https://www.arduino.cc/en/Tutorial/ArduinoToBreadboard
 *  
 *  For the ATmega328P with internal 8MHz clock
 /---------------------------------------------------------------/
 */

#include <Wire.h>
#include "ws2812.h"

#define FIRMWARE_VERSION "0.0.1"

#define PIXEL_PIN   PA7
#define PIXEL_NUM   2

#define DISABLE_PIN PA5

//Encoder Setup
#define ENC_SELECT_PIN  PA0
#define ENC_CLOCK_PIN   PF1
#define ENC_DATA_PIN    PF0

//I2C Slave Setup
#define ADDR1_PIN   PA1
#define ADDR2_PIN   PA2
#define ADDR3_PIN   PA3

//I2C Defines
#define I2C_MAG_SIG_GOOD  0
#define I2C_MAG_SIG_MID   1
#define I2C_MAG_SIG_BAD   2

#define I2C_REPORT_POSITION   0
#define I2C_REPORT_STATUS     1
#define I2C_REPORT_VERSION    2
#define I2C_REPORT_RAW        3

#define I2C_REQ_REPORT        0
#define I2C_RESET_COUNT       1
#define I2C_SET_ADDR          2
#define I2C_SET_REPORT_MODE   3
#define I2C_CLEAR_EEPROM      4

#define I2C_ENC_LED_PAR_MODE  10
#define I2C_ENC_LED_PAR_BRT   11
#define I2C_ENC_LED_PAR_RATE  12
#define I2C_ENC_LED_PAR_RGB   13
#define I2C_ENC_LED_PAR_HSV   14

//default I2C address
#define I2C_ENCODER_BASE_ADDR  30

#define MAG_GOOD_RANGE 4
#define I2C_READ_BYTES 4

#define LOOP_TIME_MICROS 1000

const byte i2c_base_address = I2C_ENCODER_BASE_ADDR;
byte i2c_address;
int i2c_response_mode = 0;

typedef union {
    volatile long val;
    byte bval[4];
}i2cLong;

i2cLong encoderCount;
i2cLong rawCount;

long count = 0;
long oldCount = 0;
long revolutions = 0;
long offset = 0;
bool offsetInitialised = false;
long avgSpeed = 0;

#define MAX_POSITION_SAMPLES 16
long positionHistory[MAX_POSITION_SAMPLES];

bool OCF = false;
bool COF = false;
bool LIN = false;
bool mINC = false;
bool mDEC = false;

int parityErrorCount = 0;

byte magStrength = I2C_MAG_SIG_BAD;

unsigned long lastLoopTime = 0;

byte addressOffset;

byte ledBrightness[] = {20,20};
byte ledMode[]       = {0,0};
byte ledRate[]       = {0,0};
byte ledSleep[]      = {0,0};

byte ledRGB[] = {255,0,0};
byte ledHSV[] = {255,255,255};

Ws2812 pixels = Ws2812(PIXEL_NUM,PIXEL_PIN);

// the setup function runs once when you press reset or power the board
void setup() {

  //Configure pins
  pinMode(ENC_SELECT_PIN, OUTPUT);
  pinMode(ADDR1_PIN,INPUT_PULLUP);
  pinMode(ADDR2_PIN,INPUT_PULLUP);
  pinMode(ADDR3_PIN,INPUT_PULLUP);
  pinMode(DISABLE_PIN,INPUT_PULLUP);
  pinMode(ENC_DATA_PIN, INPUT);
  pinMode(ENC_CLOCK_PIN, OUTPUT);
  
  digitalWrite(ENC_CLOCK_PIN, HIGH);
  digitalWrite(ENC_SELECT_PIN, HIGH);

  //Configure LEDs
  pixels.begin();

  //Initialise communication with encoder IC
  if(initEncoder() == false) {

    while(initEncoder() == false) {
     blinkLeds(1,RED); 
    }
  } else {
    blinkLeds(1,GREEN);
  }

  //Read address pins as 3-bit number
  addressOffset = !((digitalRead(ADDR3_PIN)) | (digitalRead(ADDR2_PIN) << 1) | (digitalRead(ADDR1_PIN) << 2));//0;//!digitalRead(ADDR3_PIN) + 2*(!digitalRead(ADDR2_PIN)) + 4*(!digitalRead(ADDR1_PIN));

  //Set I2C address from value
  i2c_address = i2c_base_address + addressOffset;

  //Signal I2C address
  for(int i = 0; i < (addressOffset+1); i++) {
    blinkLeds(1,(2000 / (addressOffset+1)),WHITE);
  }
  
  delay(50);

  //Join I2C bus as slave
  Wire.begin(i2c_address);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
}

// main program loop
void loop() {
  
  while(micros() - lastLoopTime < LOOP_TIME_MICROS) {
    delayMicroseconds(1); 
  }

  lastLoopTime = micros();
  
  updateEncoder();
  updateLeds();
}


////////////////////////////////////////////////////////////
//----------------------- I2C ----------------------------//
////////////////////////////////////////////////////////////

void requestEvent() {
  switch (i2c_response_mode) {
    case I2C_REPORT_POSITION:
      Wire.write(encoderCount.bval,3);
      break;
    case I2C_REPORT_STATUS:
      Wire.write(magStrength);
      break;
    case I2C_REPORT_VERSION:
      reportVersion();
      break;
    case I2C_REPORT_RAW:
      rawCount.val = count;
      Wire.write(rawCount.bval,4);
      break;
  }
}

void receiveEvent(int numBytes) {

  byte temp[5] = {0};
  int tempIndex = 0;

  while(Wire.available() > 0) {
    temp[tempIndex] = Wire.read();
    tempIndex++;
  }

  switch(temp[0]) {
    case I2C_RESET_COUNT:
      offset = encoderCount.val;
      break;
    case I2C_SET_ADDR:
      //setI2cAddress(temp[1]);
      //blinkLeds(1,WHITE);
      //restart();
      break;
    case I2C_SET_REPORT_MODE:
      i2c_response_mode = temp[1];
      break; 
    case I2C_CLEAR_EEPROM:
      //eepromClear();
      break; 
    case I2C_ENC_LED_PAR_MODE:
      setLedMode(temp[1],temp[2]);
      break;
    case I2C_ENC_LED_PAR_BRT:
      setLedBrightness(temp[1],temp[2]);
      break;  
    case I2C_ENC_LED_PAR_RGB:
      setLedRGB(temp[1],temp[2],temp[3]);
      break;   
    case I2C_ENC_LED_PAR_HSV:
      setLedHSV(temp[1],temp[2],temp[3]);
      break;   
    case I2C_ENC_LED_PAR_RATE:
      setLedRate(temp[1],temp[2]);
      break;    
  }  
}

void reportVersion() {
  // example: 1.0.0, Jul 15 2016, 21:07:40
  String versionString = FIRMWARE_VERSION ", " __DATE__ ", " __TIME__ ".";
  Wire.write(versionString.c_str());
}

////////////////////////////////////////////////////////////
//--------------------- ENCODER --------------------------//
////////////////////////////////////////////////////////////
bool initEncoder() {

  for(int i = 0; i < MAX_POSITION_SAMPLES; i++) {
    positionHistory[i] = 0;
  }
  
  return true;
}

void updateEncoder() {
  count = readPosition();

  //check if we've moved from one pole-pair to the next
  if((count-oldCount) > 2048) {
    revolutions -= 1;
  } else if((oldCount - count) > 2048) {
    revolutions += 1;
  }

  oldCount = count;

  //make the starting position 'zero'
  if(offsetInitialised == false) {
    offset = -count;
    offsetInitialised = true;
  }

  for(int i = MAX_POSITION_SAMPLES; i > 0; i--) {
    positionHistory[i] = positionHistory[i-1];
  }

  positionHistory[0] = (revolutions * 4092) + (count + offset);
  
  encoderCount.val = positionHistory[0];
  encoderCount.val = ((encoderCount.val &~((long)3 << 22)) | ((long)magStrength << 22)); //clear the upper two bits of the third byte, insert the two-bit magStrength value
}

int readPosition() {
  static uint16_t position = 0;

  //shift in our data  
  digitalWrite(ENC_SELECT_PIN, LOW);
  delayMicroseconds(1);
  byte d1 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
  byte d2 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
  byte d3 = shiftIn(ENC_DATA_PIN, ENC_CLOCK_PIN);
  digitalWrite(ENC_SELECT_PIN, HIGH);

  //check parity
  int parityCount = 0;
  
  for(int i = 0; i < 8; i++) {
    parityCount += bitRead(d1,i);
    parityCount += bitRead(d2,i);
    if(i > 5) {
      parityCount += bitRead(d3,i);
    }
  }

  if((parityCount % 2) == 0) {
    
    //get our position variable
    position = d1;
    position = position << 8;
    
    position |= d2;
    position = position >> 4;
    
    if (!(d2 & B00001000)) {
      OCF = true;
    }
    
    if (!(d2 & B00000100)) {
      COF = true;
    }
    
    LIN = bitRead(d2,1);
    mINC = bitRead(d2,0);
    mDEC = bitRead(d3,7);
    
    //determine magnetic signal strength
    if(mINC == false && mDEC == false) { magStrength = I2C_MAG_SIG_GOOD;  }
    if(mINC == true && mDEC == true && LIN == false) { magStrength = I2C_MAG_SIG_MID;  }
    if(mINC == true && mDEC == true && LIN == true) { magStrength = I2C_MAG_SIG_BAD;  }
  } else {
    parityErrorCount++;
  }

  return position;
}

//read in a byte of data from the digital input of the board.
byte shiftIn(byte data_pin, byte clock_pin)
{
  byte data = 0;

  for (int i=7; i>=0; i--)
  {
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);

    byte bit = digitalRead(data_pin);
    data |= (bit << i);
  }

  return data;
}

////////////////////////////////////////////////////////////
//---------------------- CONFIG --------------------------//
////////////////////////////////////////////////////////////

void setLedBrightness(byte led, byte brightness) {
  ledBrightness[constrain(led,0,1)] = brightness;
}

void setLedMode(byte led, byte mode) {
  ledMode[constrain(led,0,1)] = mode;
}

void setLedRate(byte led, byte rate) {
  ledRate[constrain(led,0,1)] = rate;
}

void setLedSleep(byte led, byte sleep) {
  ledSleep[constrain(led,0,1)] = sleep;
}

void setLedRGB(byte red, byte green, byte blue) {
  ledRGB[0] = red;
  ledRGB[1] = green;
  ledRGB[2] = blue;
}

void setLedHSV(byte hue, byte sat, byte val) {
  ledHSV[0] = hue;
  ledHSV[1] = sat;
  ledHSV[2] = val;
}

////////////////////////////////////////////////////////////
//------------------------ LEDS --------------------------//
////////////////////////////////////////////////////////////

void updateLeds() {
  for(int i = 0; i < PIXEL_NUM; i++) {
    switch (ledMode[i]) {
      case 0:
        if(magStrength == I2C_MAG_SIG_GOOD) { 
          pixels.setPixelColor(i, GREEN);
          //leds[i] = CRGB::Green; 
        } else if(magStrength == I2C_MAG_SIG_MID) { 
          pixels.setPixelColor(i, YELLOW);
        } else if(magStrength == I2C_MAG_SIG_BAD) {
          pixels.setPixelColor(i, RED); 
        }
        break;
      case 1:
        //pixels.setPixelColor(i, WHITE);
        break;
      case 5:
        //pixels.setPixelColor(i,ledRGB[0],ledRGB[1],ledRGB[2]);
        break;
    }
    pixels.setPixelBrightness(i,ledBrightness[i]);
  }
  pixels.show();
}

void ledTest(int pixel) {
  
  pixels.setPixelColor(pixel,255,0,0);
  pixels.show();
  delay(500);

  pixels.setPixelColor(pixel,0,255,0);
  pixels.show();
  delay(500);

  pixels.setPixelColor(pixel,0,0,255);
  pixels.show();
  delay(500);

  pixels.setPixelColor(pixel,50,50,50);
  pixels.show();
  delay(500);

  pixels.setPixelColor(pixel,100,100,100);
  pixels.show();
  delay(500);

  for(int i = 0; i < 255; i++) {
    pixels.setPixelColor(pixel, Wheel(i));
    pixels.show();
    delay(10);
  }
  
}

void blinkLeds(int times, const uint32_t rgb) {
  blinkLeds(times, 1000, rgb);
}

void blinkLeds(int times, int duration, const uint32_t rgb) {
  for(int i = 0; i < times; i++) {  
    for(int j = 0; j < PIXEL_NUM; j++) {
      pixels.setPixelColor(j,rgb);
      pixels.setPixelBrightness(j,20);//leds[j].nscale8(20);
    }
    pixels.show();
    delay(duration/2);
    for(int j = 0; j < PIXEL_NUM; j++) {
      pixels.setPixelColor(j,BLACK);
      //leds[j].nscale8(ledBrightness[j]);
    }
    pixels.show();
    delay(duration/2);
  }  
}

////////////////////////////////////////////////////////////
//------------------------ MISC --------------------------//
////////////////////////////////////////////////////////////


