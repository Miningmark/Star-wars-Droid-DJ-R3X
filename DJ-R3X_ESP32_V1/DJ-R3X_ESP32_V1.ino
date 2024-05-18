// ***************************************************************************************************
// Project: DJ-R3X (Standard version)
// Developed: Miningmark
// Version: V1 / 2023/12
// 
// Hardware:
//  - ESP32 Custom Dev Board
//  - Motor Treiber Cytron Dual Channel 10A DC
//  - Stepper Driver A4988
//  - Various servo motors
//  - Flysky FS-iA10B Transmitter
//
//    "DJ-R3X-ESP32-V1" is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    Foobar is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with Foobar.  If not, see <https://www.gnu.org/licenses/>.
//
// ***************************************************************************************************

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_PWMServoDriver.h>

// Include iBusBM Library
#include <IBusBM.h>

//#define DEBUG_RC_INPUT
//#define DEBUG_DRIVE
//#define DEBUG_SERVO
//#define DEBUG_LED
 
// Create iBus Object
IBusBM IBUS;

bool ear = true;
bool eye = true;
bool mouth = true;
bool body = true;

int bodyOffset = 0;
int ear1offset = 0;
int ear2offset = 40;
int eye1offset = 80;
int eye2offset = 81;
int mouthOffset = 82;

long bodyTime = 50000;
long bodyLastTime = micros();
long earTime = 50000;
long earLastTime = micros();
long mouthTime = 5000;
long mouthLastTime = micros();

#define NUMPIXELS_BODY 42
#define NUMPIXELS_HEAD 162
#define LED_PIN_BODY 41
#define LED_PIN_HEAD 40
int earPos1 = 0;
int earPos2 = 0;
// max Hue
#define MAXHUE 256*6
short mouthBlueIs[80];
short mouthRedIs[80];
short mouthBlueShould[80];
short mouthRedShould[80];
Adafruit_NeoPixel pixels_body(NUMPIXELS_BODY, LED_PIN_BODY, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_head(NUMPIXELS_HEAD, LED_PIN_HEAD, NEO_GRB + NEO_KHZ800);

#define RXD2 48
#define TXD2 47

#define I2C_SDA 45
#define I2C_SCL 42

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
// Define servo motor connections (expand as required)
#define SER0  12   //Servo Motor 0 on connector 12
#define SER1  13   //Servo Motor 1 on connector 13
// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;


struct servoMotor{
  int pin;
  bool reverse;
  int center;
  int min;
  int max;
  float velocidad;
  float pos;
};

struct driveMotor{
  int dirPin;
  int stepPin;
  int value;
  int speedIs;
  int speedShould;
};

struct servoMotor servo[14];
struct driveMotor motor[4];

int input[10];

int speed = 5;


//Set up time variables for Stepper motor
unsigned long currentMotorTime =  micros();


 
// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBUS.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}


void setup() {
  // Start serial monitor
  Serial.begin(115200);

  Serial2.begin(11520, SERIAL_8N1, RXD2, TXD2);
 
  // Attach iBus object to serial port
  IBUS.begin(Serial2);

  Wire.begin(I2C_SDA, I2C_SCL);
  pca9685.begin();
 
  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);

  set_params();
  setupMotor();

  //Setup LEDs
  delay(1000);
  pixels_body.begin();
  pixels_body.fill(pixels_body.Color(0, 0, 0),0,999);
  pixels_body.show();
  pixels_body.clear();
  pixels_body.show();

  pixels_head.begin();
  pixels_head.fill(pixels_head.Color(0, 0, 0),0,999);
  pixels_head.show();
  pixels_head.clear();
  pixels_head.show();

  for(int i=0; i<NUMPIXELS_BODY; i++) { // For each pixel...
    pixels_body.setPixelColor(i, 255, 255, 255);
    pixels_body.show(); 
    delay(20);
  }
  for(int i=0; i<NUMPIXELS_HEAD; i++) { // For each pixel...
    pixels_head.setPixelColor(i, 255, 255, 255);
    pixels_head.show(); 
    delay(20);
  }

  delay(1000);

  for(int i=0; i<NUMPIXELS_BODY; i++) { // For each pixel...
    pixels_body.setPixelColor(i, 0, 0, 0);
    pixels_body.show();
    delay(20);
  }
  for(int i=0; i<NUMPIXELS_HEAD; i++) { // For each pixel...
    pixels_head.setPixelColor(i, 0, 0, 0);
    pixels_head.show();
    delay(20);
  }

}

//---LOOP----------------------------------------------------------------------------------------------------
void loop() {

  
  // Note IBusBM library labels channels starting with "0"
  for (byte i = 0; i < 10; i++) {
    input[i] = readChannel(i, -100, 100, 0);
  }

  #ifdef DEBUG_RC_INPUT
    for(int i = 0; i < 10; i++){
      Serial.print("   Ch ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(input[i]);
    }
  #endif


  currentMotorTime = micros(); 
  //Serial.println(currentMotorTime - earLastTime);

//LED Body
  if(currentMotorTime - bodyLastTime >= bodyTime){
    if(input[6] == 0){  //body    input[6] == 100
    Serial.print("LEDtime: ");
    Serial.println(currentMotorTime);

    //links Rund
    pixels_body.setPixelColor(0, 128, 128, 128);
    pixels_body.setPixelColor(1, 255, 0, 0);
    pixels_body.setPixelColor(2, 0, 255, 0);
    pixels_body.setPixelColor(3, 0, 0, 255);
    pixels_body.setPixelColor(4, 255, 255, 255);
    pixels_body.setPixelColor(5, 0, 0, 0);
    pixels_body.setPixelColor(6, 0, 0, 0);
    pixels_body.setPixelColor(7, 0, 0, 128);

    //links Ecking
    pixels_body.setPixelColor(8, 0, 0, 128);
    //pixels_body.setPixelColor(9, 0, 0, 0);
    pixels_body.setPixelColor(10, 0, 128, 0);
    pixels_body.setPixelColor(11, 128, 0, 0);
    //pixels_body.setPixelColor(12, 0, 0, 0);
    //pixels_body.setPixelColor(13, 0, 0, 0);

    //mitte Rund
    pixels_body.setPixelColor(14, 128, 128, 128);
    pixels_body.setPixelColor(15, 0, 0, 0);
    pixels_body.setPixelColor(16, 0, 0, 0);
    pixels_body.setPixelColor(17, 0, 0, 0);
    pixels_body.setPixelColor(18, 0, 0, 0);
    pixels_body.setPixelColor(19, 0, 0, 0);
    pixels_body.setPixelColor(20, 0, 0, 0);
    pixels_body.setPixelColor(21, 0, 0, 0);

    //mitte Ecking
    pixels_body.setPixelColor(22, 0, 0, 128);
    //pixels_body.setPixelColor(23, 0, 0, 0);
    //pixels_body.setPixelColor(24, 0, 0, 0);
    //pixels_body.setPixelColor(25, 0, 0, 0);
    pixels_body.setPixelColor(26, 0, 128, 0);
    pixels_body.setPixelColor(27, 128, 0, 0);

    //rechts Rund
    pixels_body.setPixelColor(28, 128, 128, 128);
    pixels_body.setPixelColor(29, 0, 0, 0);
    pixels_body.setPixelColor(30, 0, 0, 0);
    pixels_body.setPixelColor(31, 0, 0, 0);
    pixels_body.setPixelColor(32, 0, 0, 0);
    pixels_body.setPixelColor(33, 0, 0, 0);
    pixels_body.setPixelColor(34, 0, 0, 0);
    pixels_body.setPixelColor(35, 0, 0, 0);

    //rechts Ecking
    //pixels_body.setPixelColor(36, 0, 0, 0);
    pixels_body.setPixelColor(37, 0, 0, 128);
    pixels_body.setPixelColor(38, 0, 128, 0);
    pixels_body.setPixelColor(39, 128, 0, 0);
    //pixels_body.setPixelColor(40, 0, 0, 0);
    //pixels_body.setPixelColor(41, 0, 0, 0);

    pixels_body.show();

    }else{
      for(int i = 0; i < 42; i++) { // For each pixel...
        pixels_body.setPixelColor(i + bodyOffset, 0, 0, 0);
      }
      pixels_body.show();
    }
    bodyLastTime = currentMotorTime;
  }

//LED Ear
  if(currentMotorTime - earLastTime >= earTime){
    if(input[6] == 0){  //ear
      for (int i = 0; i < 40; i++){
        pixels_head.setPixelColor(((i + earPos1) % 40) + ear1offset, getPixelColorHsv(i + ear1offset, (i * (MAXHUE / 40)) + ear1offset, 255, 10));
      }
      for (int i = 0; i < 40; i++){
        pixels_head.setPixelColor(((i + earPos2) % 40) + ear2offset, getPixelColorHsv(i + ear2offset, (i * (MAXHUE / 40)) + ear2offset, 255, 10));
      }
      earPos1++;
      earPos2++;
      earPos1 %= 40;
      earPos2 %= 40;
      earLastTime = currentMotorTime;
    }else{
      for(int i = 0; i < 80; i++) { // For each pixel...
        pixels_head.setPixelColor(i + ear1offset, 0, 0, 0);
      }
    }
  }

//LED Eye
  if(input[6] == 0){  //eye
    pixels_head.setPixelColor(eye1offset, 255,215,0);
    pixels_head.setPixelColor(eye2offset, 255,215,0);
  }else{
    pixels_head.setPixelColor(eye1offset, 0,0,0);
    pixels_head.setPixelColor(eye2offset, 0,0,0);
  }

//LED Mouth
  if(currentMotorTime - mouthLastTime >= mouthTime){
    if(input[6] == 0){  //mouth
      for(int i = 0; i < 80; i += 2) { // For each pixel...
        if(mouthBlueIs[i] == mouthBlueShould[i]){
          int tmp = random(0, 255);
          mouthBlueShould[i] = tmp;
        }
        if(mouthBlueIs[i] < mouthBlueShould[i]){
          mouthBlueIs[i] += 1;
        }else{
          mouthBlueIs[i] -= 1;
        }
        if(mouthRedIs[i] == mouthRedShould[i]){
          int tmp = random(0, 255);
          mouthRedShould[i] = tmp;
        }
        if(mouthRedIs[i] < mouthRedShould[i]){
          mouthRedIs[i] += 1;
        }else{
          mouthRedIs[i] -= 1;
        }

        #ifdef DEBUG_LED
          for(int i = 0; i < 10; i++){
            Serial.print("Red is: ");
            Serial.print(mouthRedIs[i]);
            Serial.print("   Blue is: ");
            Serial.println(mouthBlueIs[i]);
          }
        #endif

        //---------------------------------------------------------------------------------------------------------------------------
        pixels_head.setPixelColor(i + mouthOffset, pixels_head.Color(mouthRedIs[i], 0, mouthBlueIs[i]));
      }
    }else{
      for(int i = 0; i < 80; i++) { // For each pixel...
        pixels_head.setPixelColor(i + mouthOffset, pixels_head.Color(0, 0, 0));
      }
    }
    mouthLastTime = currentMotorTime;
  }
  pixels_head.show();


/*
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, 80, 600);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    // Print to serial monitor
    Serial.print("Motor 0 = ");
    Serial.println(posDegrees);
    delay(30);
  }
  // Move Motor 0 from 180 to 0 degrees
  for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, 80, 600);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    // Print to serial monitor
    Serial.print("Motor 0 = ");
    Serial.println(posDegrees);
    delay(30);
  }
*/

  /*

  Ch 6: -100 -> Light off     100 -> Light on
  Ch 7: -100 -> Arme   100 -> Head
  Ch 8: -100 -> Head Yaw/drehen   0 -> Head Roll/Pitch   100 -> Head Shield  
  Ch 8: -100 -> Arm3   0 -> Arm 2   100 -> Arm 1
  Ch 9: -100 Fahren linker stick    100 Arme Bewegen/Kopf Bewegen

  */


  //calculate Drive Speed and Direction
  drive();

  if(input[9] == 100){

    //Arme
    if(input[7] == -100){

    //Arm auswahl
      if(input[8] == -100){
        //Arm 1
        servoCalcEndless(0, input[3]);
        servoCalc(1, input[2]);
        servoCalc(2, input[5]);
      }else if(input[8] == 0){
        //Arm 2
        servoCalcEndless(3, input[3]);
        servoCalc(4, input[2]);
        servoCalc(5, input[5]);
      }else{
        //Arm 3
        servoCalcEndless(6, input[3]);
        servoCalc(7, input[2]);
        servoCalc(8, input[5]);
      }
    
    }else{

    //Head 
      if(input[8] == -100){
        servoCalc(9, input[3]);
        servoCalc(10, input[2]);
      }else if(input[8] == 0){
        servoCalc(11,input[2]);
        servoCalc(12,input[3]);
      }else{
        servoCalc(13,input[2]);
      }
    }
  }

  
  #if defined(DEBUG_DRIVE) || defined(DEBUG_STEPPER) || defined(DEBUG_RC_INPUT) || defined(DEBUG_SERVO)
    Serial.println("");
  #endif

  delay(10);
}


void servoCalcEndless(int number, int value){
  int pwm;

  pwm = mapF(value, -100, 100, 80, 500);

  // Write to PCA9685
  pca9685.setPWM(servo[number].pin, 0, pwm);
  
  #ifdef DEBUG_SERVO
  Serial.print("  Servo Endless: ");
  Serial.print(value);
  Serial.print("  PWM: ");
  Serial.print(pwm);
  #endif

}


void servoCalc(int number, int value){

  float temp;
  int pwm;

  if(servo[number].reverse == 0){
    servo[number].pos = map(value, -100, 100, servo[number].min, servo[number].max);
  }else{
    servo[number].pos = map(value, -100, 100, servo[number].max, servo[number].min);
  }

/*
  if(value > 1){
    temp = mapF(value, 0, 100, 0, servo[number].velocidad);
    if(servo[number].pos + temp <= servo[number].max){
      servo[number].pos += temp;
    }else{
      servo[number].pos = servo[number].max;
    }
  }else{
    temp = mapF(value, -100, 0, servo[number].velocidad, 0);
    if(servo[number].pos - temp >= servo[number].min){
      servo[number].pos -= temp;
    }else{
      servo[number].pos = servo[number].min;
    }
  }
*/
  // Determine PWM pulse width
  pwm = map(servo[number].pos, 0, 180, 80, 500);  
  // Write to PCA9685
  pca9685.setPWM(servo[number].pin, 0, pwm);

  #ifdef DEBUG_SERVO
  Serial.print("  Move: ");
  Serial.print(temp);
  Serial.print("  PWM: ");
  Serial.print(pwm);
  #endif

}

//Custom MAP function for float values
float mapF(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void drive(){

  int maxSpe = 200;

  if(input[9] == -100){
    //Drive with left und right stick
    motor[0].value = (input[1] + input[0] + input[3]);
    motor[1].value = (input[1] - input[0] - input[3]);
    motor[2].value = (input[1] + input[0] - input[3]);
    motor[3].value = (input[1] - input[0] + input[3]);
    maxSpe += 100;
  }else{
    //Drive only with right stick
    motor[0].value = (input[1] + input[0]);
    motor[1].value = (input[1] - input[0]);
    motor[2].value = (input[1] + input[0]);
    motor[3].value = (input[1] - input[0]);
  }

  for(int i = 0; i <  4; i++){
    motor[i].speedShould = map (motor[i].value,maxSpe*(-1),maxSpe,-255,255);

    if(motor[i].speedShould == motor[i].speedIs){
      motor[i].speedIs = motor[i].speedShould;
    }else if(motor[i].speedShould > motor[i].speedIs){
      motor[i].speedIs += speed;
    }else if(motor[i].speedShould < motor[i].speedIs){
      motor[i].speedIs -= speed;
    }

    if(motor[i].speedIs < 0){
      digitalWrite(motor[i].dirPin, LOW);
      analogWrite(motor[i].stepPin,motor[i].speedIs*(-1)); //increase the speed of the motor from 0 to 255
    }else{
      digitalWrite(motor[i].dirPin, HIGH);
      analogWrite(motor[i].stepPin,motor[i].speedIs); //increase the speed of the motor from 0 to 255
    }
  }

/*
  motor[1].speedShould = map (motor[1].value,maxSpe*(-1),maxSpe,-255,255);
  motor[2].speedShould = map (motor2,maxSpe*(-1),maxSpe,-255,255); 
  motor[3].speedShould = map (motor3,maxSpe*(-1),maxSpe,-255,255); 
  motor[4].speedShould = map (motor4,maxSpe*(-1),maxSpe,-255,255);

  if(motorSpeed1Should == motorSpeed1Is){
    motorSpeed1Is = motorSpeed1Should;
  }else if(motorSpeed1Should > motorSpeed1Is){
    motorSpeed1Is += speed;
  }else if(motorSpeed1Should < motorSpeed1Is){
    motorSpeed1Is -= speed;
  }

  if(motorSpeed2Should == motorSpeed2Is){
    motorSpeed2Is = motorSpeed2Should;
  }else if(motorSpeed2Should > motorSpeed2Is){
    motorSpeed2Is += speed;
  }else if(motorSpeed2Should < motorSpeed2Is){
    motorSpeed2Is -= speed;
  }

  if(motorSpeed3Should == motorSpeed3Is){
    motorSpeed3Is = motorSpeed3Should;
  }else if(motorSpeed3Should > motorSpeed3Is){
    motorSpeed3Is += speed;
  }else if(motorSpeed3Should < motorSpeed3Is){
    motorSpeed3Is -= speed;
  }

  if(motorSpeed4Should == motorSpeed4Is){
    motorSpeed4Is = motorSpeed4Should;
  }else if(motorSpeed4Should > motorSpeed4Is){
    motorSpeed4Is += speed;
  }else if(motorSpeed4Should < motorSpeed4Is){
    motorSpeed4Is -= speed;
  }

  if(motorSpeed1Is < 0){
    digitalWrite(dir1pin, LOW);
    analogWrite(spe1pin,motorSpeed1Is*(-1)); //increase the speed of the motor from 0 to 255
  }else{
    digitalWrite(dir1pin, HIGH);
    analogWrite(spe1pin,motorSpeed1Is); //increase the speed of the motor from 0 to 255
  }

  if(motorSpeed2Is < 0){
    digitalWrite(dir2pin, LOW);
    analogWrite(spe2pin,motorSpeed2Is*(-1)); //increase the speed of the motor from 0 to 255
  }else{
    digitalWrite(dir2pin, HIGH);
    analogWrite(spe2pin,motorSpeed2Is); //increase the speed of the motor from 0 to 255
  }

  if(motorSpeed3Is < 0){
    digitalWrite(dir3pin, LOW);
    analogWrite(spe3pin,motorSpeed3Is*(-1)); //increase the speed of the motor from 0 to 255
  }else{
    digitalWrite(dir3pin, HIGH);
    analogWrite(spe3pin,motorSpeed3Is); //increase the speed of the motor from 0 to 255
  }

  if(motorSpeed4Is < 0){
    digitalWrite(dir4pin, LOW);
    analogWrite(spe4pin,motorSpeed4Is*(-1)); //increase the speed of the motor from 0 to 255
  }else{
    digitalWrite(dir4pin, HIGH);
    analogWrite(spe4pin,motorSpeed4Is); //increase the speed of the motor from 0 to 255
  }
*/


  #ifdef DEBUG_DRIVE
    Serial.print("Motor1: ");
    Serial.print(motor[1].speedIs);
    Serial.print("  Motor2: ");
    Serial.print(motor[2].speedIs);
    Serial.print("  Motor3: ");
    Serial.print(motor[3].speedIs);
    Serial.print("  Motor4: ");
    Serial.print(motor[4].speedIs);
    Serial.print("  Speed1: ");
    Serial.print(motor[1].value);
    Serial.print("  Speed2: ");
    Serial.print(motor[2].value);
    Serial.print("  Speed3: ");
    Serial.print(motor[3].value);
    Serial.print("  Speed4: ");
    Serial.print(motor[4].value);
  #endif
}

