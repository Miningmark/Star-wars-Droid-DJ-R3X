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
//#define DEBUG_STEPPER
//#define DEBUG_SERVO
//#define DEBUG_LED
 
// Create iBus Object
IBusBM IBUS;

bool ear = true;
bool eye = true;
bool mouth = true;
bool body = true;

int bodyOffset = 0;
int ear1offset = 42;
int ear2offset = 82;
int eye1offset = 122;
int eye2offset = 123;
int mouthOffset = 124;

long bodyTime = 50000;
long bodyLastTime = micros();
long earTime = 50000;
long earLastTime = micros();
long mouthTime = 5000;
long mouthLastTime = micros();

#define NUMPIXELS 204
#define LEDPIN 41
int earPos1 = 0;
int earPos2 = 0;
// max Hue
#define MAXHUE 256*6
short mouthBlueIs[80];
short mouthRedIs[80];
short mouthBlueShould[80];
short mouthRedShould[80];
Adafruit_NeoPixel pixels(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#define RXD2 48
#define TXD2 47

#define I2C_SDA 45
#define I2C_SCL 42

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
// Define servo motor connections (expand as required)
#define SER0  12   //Servo Motor 0 on connector 12
#define SER1  13   //Servo Motor 0 on connector 13
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

struct stepperMotor{
  int dirPin;
  int stepPin;
  int stepsPerRound;
  long previousMotorTime;
  int speed;
  int speedMin;
  int speedMax;
  int speedAcc;
  int pos;
  int posMin;
  int posMax;
};

struct servoMotor servo[9];
struct driveMotor motor[4];
struct stepperMotor stepper[5];

/*
int dir1pin = 1;
int spe1pin = 2;
int dir2pin = 3;
int spe2pin = 4;
int dir3pin = 5;
int spe3pin = 6;
int dir4pin = 7;
int spe4pin = 8;
*/

/*
int dir5pin = 9;
int step5pin = 10;
int steps_per_rev = 800;
*/

int input[10];

/*
int motor5Min = 0;
int motor5Max = 1000;

int motor1 = 0;
int motor2 = 0;
int motor3 = 0;
int motor4 = 0;
int motor5 = 0;

int motor5Should = 0;

int motorSpeed1Is = 0;
int motorSpeed2Is = 0;
int motorSpeed3Is = 0;
int motorSpeed4Is = 0;
int motorSpeed5Is = 0;

int motorSpeed1Should = 0;
int motorSpeed2Should = 0;
int motorSpeed3Should = 0;
int motorSpeed4Should = 0;
int motorSpeed5Should = 0;

int stepperSpeedMin = 1500;
int stepperSpeedMax = 100;
int stepperSpeedAcc = 25;
int stepperSpeed = 0;

*/
int speed = 5;


//Set up time variables for Stepper motor
long currentMotorTime =  micros();


 
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
  pixels.begin();
  pixels.fill(pixels.Color(0, 0, 0),0,999);
  pixels.show();
  pixels.clear();
  pixels.show();

  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, 255, 255, 255);
    pixels.show(); 
    delay(20);
  }

  delay(1000);

  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    pixels.show();
    delay(20);
  }

}

void loop() {


  currentMotorTime = micros(); 
  //Serial.println(currentMotorTime - earLastTime);
  if(currentMotorTime - earLastTime >= earTime){
    if(ear){
      for (int i = 0; i < 40; i++){
        pixels.setPixelColor(((i + earPos1) % 40) + ear1offset, getPixelColorHsv(i + ear1offset, (i * (MAXHUE / 40)) + ear1offset, 255, 10));
      }
      for (int i = 0; i < 40; i++){
        pixels.setPixelColor(((i + earPos2) % 40) + ear2offset, getPixelColorHsv(i + ear2offset, (i * (MAXHUE / 40)) + ear2offset, 255, 10));
      }
      earPos1++;
      earPos2++;
      earPos1 %= 40;
      earPos2 %= 40;
      earLastTime = currentMotorTime;
    }else{
      for(int i = 0; i < 80; i++) { // For each pixel...
        pixels.setPixelColor(i + ear1offset, pixels.Color(0, 0, 0));
      }
    }
  }

  if(eye){
    pixels.setPixelColor(eye1offset, pixels.Color(255,215,0));
    pixels.setPixelColor(eye2offset, pixels.Color(255,215,0));
  }else{
    pixels.setPixelColor(eye1offset, pixels.Color(0,0,0));
    pixels.setPixelColor(eye2offset, pixels.Color(0,0,0));
  }

  if(currentMotorTime - mouthLastTime >= mouthTime){
    if(mouth){
      for(int i = 0; i < 80; i++) { // For each pixel...
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
        //Serial.print("Red ist: ");
        //Serial.print(mouthRedIs[i]);
        //Serial.print("   Blue ist: ");
        //Serial.println(mouthBlueIs[i]);

        //---------------------------------------------------------------------------------------------------------------------------
        pixels.setPixelColor(i + mouthOffset, pixels.Color(mouthRedIs[i], 0, mouthBlueIs[i]));
      }
    }else{
      for(int i = 0; i < 80; i++) { // For each pixel...
        pixels.setPixelColor(i + mouthOffset, pixels.Color(0, 0, 0));
      }
    }
    mouthLastTime = currentMotorTime;
  }
  pixels.show();


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
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
 
    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, 80, 600);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    // Print to serial monitor
    Serial.print("Motor 1 = ");
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
  // Move Motor 0 from 180 to 0 degrees
  for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, 80, 600);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    // Print to serial monitor
    Serial.print("Motor 1 = ");
    Serial.println(posDegrees);
    delay(30);
  }
*/

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

  /*


  Ch 8: -100 -> Arm3   0 -> Arm 2   100 -> Arm 1
  Ch 9: -100 Fahren linker stick    100 Arme Bewegen

  */


  //calculate Drive Speed and Direction
  drive();

  //Arm auswahl
  if(input[9] == 100){
    if(input[8] == -100){
      stepperCalc(0);
      servoCalc(0);
    }else if(input[8] == 0){
      stepperCalc(1);
      servoCalc(1);
    }else{
      stepperCalc(2);
    }
  }
  
  //Head Movement
  if(input[9] == -100){
    if(input[8] == 0){
      stepperCalc(3);
      servoCalc(0);
    }else if(input[8] == 100){
      stepperCalc(4);
      servoCalc(1);
    }
  }

  
  #if defined(DEBUG_DRIVE) || defined(DEBUG_STEPPER) || defined(DEBUG_RC_INPUT) || defined(DEBUG_SERVO)
    Serial.println("");
  #endif

  delay(1);
  //delayMicroseconds(800);
}


void servoCalc(int number){

  float temp;
  int pwm;

  if(input[3] > 1){
    temp = mapF(input[3], 0, 100, 0, servo[number].velocidad);
    if(servo[number].pos + temp <= servo[number].max){
      servo[number].pos += temp;
    }else{
      servo[number].pos = servo[number].max;
    }
  }else{
    temp = mapF(input[3], -100, 0, servo[number].velocidad, 0);
    if(servo[number].pos - temp >= servo[number].min){
      servo[number].pos -= temp;
    }else{
      servo[number].pos = servo[number].min;
    }
  }

  // Determine PWM pulse width
  pwm = map(servo[number].pos, servo[number].min, servo[number].max, 80, 600);  
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


void stepperCalc(int number){
  //number is the number of the Stepper(0-4)
  int i = number;

  if(input[2] > 1){
    stepper[i].speed = map (input[2],0,100,stepper[i].speedMin,stepper[i].speedMax);
    if(stepper[i].pos + 1 < stepper[i].posMax){
      digitalWrite(stepper[i].dirPin, HIGH);

      currentMotorTime = micros(); 
      if (currentMotorTime - stepper[i].previousMotorTime > stepper[i].speed){
        digitalWrite(stepper[i].stepPin, HIGH);
        stepper[i].previousMotorTime = currentMotorTime;
        stepper[i].pos++;
      }  

      digitalWrite(stepper[i].stepPin, LOW);

    }
  }else if(input[2] < -1){
    stepper[i].speed = map (input[2],-100,0,stepper[i].speedMax,stepper[i].speedMin);
    if(stepper[i].pos - 1 > stepper[i].posMin){
      digitalWrite(stepper[i].dirPin, LOW);

      currentMotorTime = micros(); 
      if (currentMotorTime - stepper[i].previousMotorTime > stepper[i].speed){
        digitalWrite(stepper[i].stepPin, HIGH);
        stepper[i].previousMotorTime = currentMotorTime;
        stepper[i].pos--;
      }  

      digitalWrite(stepper[i].stepPin, LOW);

    }
  }
  
/*
  stepper[0].posShould = map (input[2],-100,100,stepper[0].posMin,stepper[0].posMax);

  if(stepper[0].posShould != stepper[0].pos){
    if(stepper[0].posShould > stepper[0].pos){
      
      int temp = stepper[0].posShould - stepper[0].pos;
      if(temp > stepper[0].speed){
        if(stepper[0].speed < ((stepper[0].speedMin - stepper[0].speedMax) / stepper[0].speedAcc)){   // ((stepper[0].speedMin - stepper[0].speedMax) / stepper[0].speedAcc)
          stepper[0].speed++;
        }
      }else{
        if(stepper[0].speed > 0){
          stepper[0].speed--;
        }
      }
      
      int duration = stepper[0].speedMin - (stepper[0].speed * stepper[0].speedAcc);

      digitalWrite(dir5pin, HIGH);

      currentMotorTime = micros(); 
      if (currentMotorTime - previousMotorTime > duration){
        digitalWrite(step5pin, HIGH);
        previousMotorTime = currentMotorTime;
        stepper[0].pos++;
      }  
      digitalWrite(step5pin, LOW);

      
    }else{

      int temp = stepper[0].pos - stepper[0].posShould;
      if(temp > stepper[0].speed){
        if(stepper[0].speed < ((stepper[0].speedMin - stepper[0].speedMax) / stepper[0].speedAcc)){
          stepper[0].speed++;
        }
      }else{
        if(stepper[0].speed > 0){
          stepper[0].speed--;
        }
      }
      
      int duration = stepper[0].speedMin - stepper[0].speed * stepper[0].speedAcc;

      digitalWrite(dir5pin, LOW);

      currentMotorTime = micros(); 
      if (currentMotorTime - previousMotorTime > duration){
        digitalWrite(step5pin, HIGH);
        previousMotorTime = currentMotorTime;
        stepper[0].pos--;
      }  
      digitalWrite(step5pin, LOW);

    }
  }
*/

  #ifdef DEBUG_STEPPER
    Serial.print("  Input Ch3: ");
    Serial.print(input[2]);
    Serial.print("  stepperSpeed: ");
    Serial.print(stepper[0].speed);
    Serial.print("  stepperPos: ");
    Serial.print(stepper[0].pos);
  #endif

}


void drive(){

  int maxSpe = 200;

  if(input[9] == -100){
    //Drive with left und right stick
    motor[1].value = (input[1] + input[0] + input[3]);
    motor[2].value = (input[1] - input[0] - input[3]);
    motor[3].value = (input[1] + input[0] - input[3]);
    motor[4].value = (input[1] - input[0] + input[3]);
    maxSpe += 100;
  }else{
    //Drive only with right stick
    motor[1].value = (input[1] + input[0]);
    motor[2].value = (input[1] - input[0]);
    motor[3].value = (input[1] + input[0]);
    motor[4].value = (input[1] - input[0]);
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
  Serial.print(motor1);
  Serial.print("  Motor2: ");
  Serial.print(motor2);
  Serial.print("  Motor3: ");
  Serial.print(motor3);
  Serial.print("  Motor4: ");
  Serial.print(motor4);
  Serial.print("  Speed1: ");
  Serial.print(motorSpeed1Is);
  Serial.print("  Speed2: ");
  Serial.print(motorSpeed2Is);
  Serial.print("  Speed3: ");
  Serial.print(motorSpeed3Is);
  Serial.print("  Speed4: ");
  Serial.print(motorSpeed4Is);
  #endif
}









