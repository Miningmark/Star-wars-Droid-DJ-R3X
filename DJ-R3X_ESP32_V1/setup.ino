// Setup your droid


void set_params(){

  // *****************************************************************************  
  // SERVO 1
  // *****************************************************************************
  servo[0].pin=12;                 //Pin auf PCA9685
  servo[0].reverse = 0;           // Turning sense
  servo[0].center=90;             // Center Angle
  servo[0].min=60;                // Center Angle
  servo[0].max=120;               // Center Angle
  servo[0].velocidad = 0.5;      // velocity move
  servo[0].pos = servo[0].center;  //Current position
  // *****************************************************************************  
  // SERVO 2
  // *****************************************************************************
  servo[1].pin=13;                 //Pin auf PCA9685
  servo[1].reverse = 0;           // Turning sense
  servo[1].center=90;             // Center Angle
  servo[1].min=60;                // Center Angle
  servo[1].max=120;               // Center Angle
  servo[1].velocidad = 0.25;      // velocity move
  servo[1].pos = servo[1].center;  //Current position
  // *****************************************************************************  
  // SERVO 3
  // *****************************************************************************
  servo[2].pin=14;                 //Pin auf PCA9685
  servo[2].reverse = 0;          // Turning sense
  servo[2].center=90;            // Center Angle
  servo[2].min=60;                // Center Angle
  servo[2].max=120;               // Center Angle
  servo[2].velocidad = 0.50;     // velocity move
  servo[2].pos = servo[2].center;  //Current position
  // *****************************************************************************  
  // SERVO 4
  // *****************************************************************************
  servo[3].pin=15;                 //Pin auf PCA9685
  servo[3].reverse = 0;          // Turning sense
  servo[3].center=90;            // Center Angle
  servo[3].min=60;                // Center Angle
  servo[3].max=120;               // Center Angle
  servo[3].velocidad = 0.25;     // velocity move
  servo[3].pos = servo[3].center;  //Current position
  // *****************************************************************************  
  // SERVO 5
  // *****************************************************************************
  servo[4].pin=11;                 //Pin auf PCA9685
  servo[4].reverse = 0;          // Turning sense
  servo[4].center=90;            // Center Angle
  servo[4].min=60;                // Center Angle
  servo[4].max=120;               // Center Angle
  servo[4].velocidad = 0.25;     // velocity move
  servo[4].pos = servo[4].center;  //Current position
  // *****************************************************************************  
  // SERVO 6
  // *****************************************************************************
  servo[5].pin=10;                 //Pin auf PCA9685
  servo[5].reverse = 0;          // Turning sense
  servo[5].center=90;            // Center Angle
  servo[5].min=60;                // Center Angle
  servo[5].max=120;               // Center Angle
  servo[5].velocidad = 0.25;     // velocity move
  servo[5].pos = servo[5].center;  //Current position
  // *****************************************************************************  
  // SERVO 7
  // *****************************************************************************
  servo[6].pin=9;                 //Pin auf PCA9685
  servo[6].reverse = 0;          // Turning sense
  servo[6].center=90;            // Center Angle
  servo[6].min=60;                // Center Angle
  servo[6].max=120;               // Center Angle
  servo[6].velocidad = 0.25;     // velocity move
  servo[6].pos = servo[6].center;  //Current position
  // *****************************************************************************  
  // SERVO 8
  // *****************************************************************************
  servo[7].pin=8;                 //Pin auf PCA9685
  servo[7].reverse = 0;          // Turning sense
  servo[7].center=90;            // Center Angle
  servo[7].min=60;                // Center Angle
  servo[7].max=120;               // Center Angle
  servo[7].velocidad = 0.25;     // velocity move
  servo[7].pos = servo[7].center;  //Current position
  // *****************************************************************************  
  // SERVO 9
  // *****************************************************************************
  servo[8].pin=7;                 //Pin auf PCA9685
  servo[8].reverse = 0;          // Turning sense
  servo[8].center=90;            // Center Angle
  servo[8].min=60;                // Center Angle
  servo[8].max=120;               // Center Angle
  servo[8].velocidad = 0.25;     // velocity move
  servo[8].pos = servo[8].center;  //Current position



  // *****************************************************************************  
  // Drive Motor 1
  // *****************************************************************************
  motor[0].dirPin = 2;
  motor[0].stepPin = 1;
  motor[0].value = 0;
  motor[0].speedIs = 0;
  motor[0].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 2
  // *****************************************************************************
  motor[1].dirPin = 4;
  motor[1].stepPin = 3;
  motor[1].value = 0;
  motor[1].speedIs = 0;
  motor[1].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 3
  // *****************************************************************************
  motor[2].dirPin = 6;
  motor[2].stepPin = 5;
  motor[2].value = 0;
  motor[2].speedIs = 0;
  motor[2].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 4
  // *****************************************************************************
  motor[3].dirPin = 8;
  motor[3].stepPin = 7;
  motor[3].value = 0;
  motor[3].speedIs = 0;
  motor[3].speedShould = 0;



  // *****************************************************************************  
  // Stepper Arm 1
  // *****************************************************************************
  stepper[0].dirPin = 10;
  stepper[0].stepPin = 9;
  stepper[0].stepsPerRound = 800;
  stepper[0].previousMotorTime = micros();
  stepper[0].speed = 0;
  stepper[0].speedMin = 2000;
  stepper[0].speedMax = 100;
  stepper[0].speedAcc = 25;
  stepper[0].pos = 0;
  stepper[0].posMin = 0;
  stepper[0].posMax = 1000;
  // *****************************************************************************  
  // Stepper Arm 2
  // *****************************************************************************
  stepper[1].dirPin = 12;
  stepper[1].stepPin = 11;
  stepper[1].stepsPerRound = 800;
  stepper[1].previousMotorTime = micros();
  stepper[1].speed = 0;
  stepper[1].speedMin = 1500;
  stepper[1].speedMax = 100;
  stepper[1].speedAcc = 25;
  stepper[1].pos = 0;
  stepper[1].posMin = 0;
  stepper[1].posMax = 1000;
  // *****************************************************************************  
  // Stepper Arm 3
  // *****************************************************************************
  stepper[2].dirPin = 14;
  stepper[2].stepPin = 13;
  stepper[2].stepsPerRound = 800;
  stepper[2].previousMotorTime = micros();
  stepper[2].speed = 0;
  stepper[2].speedMin = 1500;
  stepper[2].speedMax = 100;
  stepper[2].speedAcc = 25;
  stepper[2].pos = 0;
  stepper[2].posMin = 0;
  stepper[2].posMax = 1000;
  // *****************************************************************************  
  // Stepper Kopf Yaw
  // *****************************************************************************
  stepper[3].dirPin = 18;
  stepper[3].stepPin = 17;
  stepper[3].stepsPerRound = 800;
  stepper[3].previousMotorTime = micros();
  stepper[3].speed = 0;
  stepper[3].speedMin = 1500;
  stepper[3].speedMax = 100;
  stepper[3].speedAcc = 25;
  stepper[3].pos = 0;
  stepper[3].posMin = 0;
  stepper[3].posMax = 1000;
  // *****************************************************************************  
  // Stepper Kopf Height
  // *****************************************************************************
  stepper[4].dirPin = 20;
  stepper[4].stepPin = 19;
  stepper[4].stepsPerRound = 800;
  stepper[4].previousMotorTime = micros();
  stepper[4].speed = 0;
  stepper[4].speedMin = 1500;
  stepper[4].speedMax = 100;
  stepper[4].speedAcc = 25;
  stepper[4].pos = 0;
  stepper[4].posMin = 0;
  stepper[4].posMax = 1000;

}



void setupMotor(){

//Servo Motors


//Drive Motors
  pinMode(motor[0].dirPin,OUTPUT);
  pinMode(motor[0].stepPin,OUTPUT);
  pinMode(motor[1].dirPin,OUTPUT);
  pinMode(motor[1].stepPin,OUTPUT);
  pinMode(motor[2].dirPin,OUTPUT);
  pinMode(motor[2].stepPin,OUTPUT);
  pinMode(motor[3].dirPin,OUTPUT);
  pinMode(motor[3].stepPin,OUTPUT);

//Stepper Motors
  pinMode(stepper[0].dirPin,OUTPUT);
  pinMode(stepper[0].stepPin,OUTPUT);
  pinMode(stepper[1].dirPin,OUTPUT);
  pinMode(stepper[1].stepPin,OUTPUT);
  pinMode(stepper[2].dirPin,OUTPUT);
  pinMode(stepper[2].stepPin,OUTPUT);
  pinMode(stepper[3].dirPin,OUTPUT);
  pinMode(stepper[3].stepPin,OUTPUT);
  pinMode(stepper[4].dirPin,OUTPUT);
  pinMode(stepper[4].stepPin,OUTPUT);

}

