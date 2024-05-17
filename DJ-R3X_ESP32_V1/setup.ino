// Setup your droid


void set_params(){

  // *****************************************************************************  
  // SERVO 1  - Arm 1 Drehen
  // *****************************************************************************
  servo[0].pin=1;                 //Pin auf PCA9685
  servo[0].reverse = 0;           // Turning sense
  servo[0].center=90;             // Center Angle
  servo[0].min=60;                // Center Angle
  servo[0].max=120;               // Center Angle
  servo[0].velocidad = 0.5;      // velocity move
  servo[0].pos = servo[0].center;  //Current position
  // *****************************************************************************  
  // SERVO 2  - Arm 1 Schulter
  // *****************************************************************************
  servo[1].pin=2;                 //Pin auf PCA9685
  servo[1].reverse = 0;           // Turning sense
  servo[1].center=90;             // Center Angle
  servo[1].min=80;                // Center Angle
  servo[1].max=100;               // Center Angle
  servo[1].velocidad = 0.25;      // velocity move
  servo[1].pos = servo[1].center;  //Current position
  // *****************************************************************************  
  // SERVO 3  - Arm 1 hand
  // *****************************************************************************
  servo[2].pin=3;                 //Pin auf PCA9685
  servo[2].reverse = 0;          // Turning sense
  servo[2].center=90;            // Center Angle
  servo[2].min=80;                // Center Angle
  servo[2].max=170;               // Center Angle
  servo[2].velocidad = 0.50;     // velocity move
  servo[2].pos = servo[2].center;  //Current position
  // *****************************************************************************  
  // SERVO 4  - Arm 2 Drehen
  // *****************************************************************************
  servo[3].pin=4;                 //Pin auf PCA9685
  servo[3].reverse = 0;          // Turning sense
  servo[3].center=90;            // Center Angle
  servo[3].min=60;                // Center Angle
  servo[3].max=120;               // Center Angle
  servo[3].velocidad = 0.25;     // velocity move
  servo[3].pos = servo[3].center;  //Current position
  // *****************************************************************************  
  // SERVO 5  - Arm 2 Schulter
  // *****************************************************************************
  servo[4].pin=5;                 //Pin auf PCA9685
  servo[4].reverse = 1;          // Turning sense
  servo[4].center=80;            // Center Angle
  servo[4].min=50;                // Center Angle
  servo[4].max=90;               // Center Angle
  servo[4].velocidad = 0.25;     // velocity move
  servo[4].pos = servo[4].center;  //Current position
  // *****************************************************************************  
  // SERVO 6  - Arm 2 Hand
  // *****************************************************************************
  servo[5].pin=6;                 //Pin auf PCA9685
  servo[5].reverse = 0;          // Turning sense
  servo[5].center=90;            // Center Angle
  servo[5].min=0;                // Center Angle
  servo[5].max=180;               // Center Angle
  servo[5].velocidad = 0.25;     // velocity move
  servo[5].pos = servo[5].center;  //Current position
  // *****************************************************************************  
  // SERVO 7 - Arm 3 Drehen
  // *****************************************************************************
  servo[6].pin=7;                 //Pin auf PCA9685
  servo[6].reverse = 0;          // Turning sense
  servo[6].center=90;            // Center Angle
  servo[6].min=70;                // Center Angle
  servo[6].max=110;               // Center Angle
  servo[6].velocidad = 0.25;     // velocity move
  servo[6].pos = servo[6].center;  //Current position
  // *****************************************************************************  
  // SERVO 8  - Arm 3 Schulter
  // *****************************************************************************
  servo[7].pin=8;                 //Pin auf PCA9685
  servo[7].reverse = 0;          // Turning sense
  servo[7].center=110;            // Center Angle
  servo[7].min=90;                // Center Angle
  servo[7].max=150;               // Center Angle
  servo[7].velocidad = 0.25;     // velocity move
  servo[7].pos = servo[7].center;  //Current position
  // *****************************************************************************  
  // SERVO 9  - Arm 3 Hand
  // *****************************************************************************
  servo[8].pin=9;                 //Pin auf PCA9685
  servo[8].reverse = 0;          // Turning sense
  servo[8].center=90;            // Center Angle
  servo[8].min=70;                // Center Angle
  servo[8].max=110;               // Center Angle
  servo[8].velocidad = 0.25;     // velocity move
  servo[8].pos = servo[8].center;  //Current position
  // *****************************************************************************  
  // SERVO 10  - Head Drehen
  // *****************************************************************************
  servo[9].pin=10;                 //Pin auf PCA9685
  servo[9].reverse = 0;          // Turning sense
  servo[9].center=90;            // Center Angle
  servo[9].min=0;                // Center Angle
  servo[9].max=180;               // Center Angle
  servo[9].velocidad = 0.25;     // velocity move
  servo[9].pos = servo[8].center;  //Current position
  // *****************************************************************************  
  // SERVO 11  - Head Höhe
  // *****************************************************************************
  servo[10].pin=11;                 //Pin auf PCA9685
  servo[10].reverse = 1;          // Turning sense
  servo[10].center=120;            // Center Angle
  servo[10].min=60;                // Center Angle
  servo[10].max=180;               // Center Angle
  servo[10].velocidad = 0.25;     // velocity move
  servo[10].pos = servo[8].center;  //Current position
  // *****************************************************************************  
  // SERVO 12  - Head Pitch
  // *****************************************************************************
  servo[11].pin=12;                 //Pin auf PCA9685
  servo[11].reverse = 0;          // Turning sense
  servo[11].center=90;            // Center Angle
  servo[11].min=70;                // Center Angle
  servo[11].max=110;               // Center Angle
  servo[11].velocidad = 0.25;     // velocity move
  servo[11].pos = servo[8].center;  //Current position
  // *****************************************************************************  
  // SERVO 13  - Head Roll
  // *****************************************************************************
  servo[12].pin=13;                 //Pin auf PCA9685
  servo[12].reverse = 0;          // Turning sense
  servo[12].center=90;            // Center Angle
  servo[12].min=70;                // Center Angle
  servo[12].max=110;               // Center Angle
  servo[12].velocidad = 0.25;     // velocity move
  servo[12].pos = servo[8].center;  //Current position
  // *****************************************************************************  
  // SERVO 14  - Head Shield
  // *****************************************************************************
  servo[13].pin=14;                 //Pin auf PCA9685
  servo[13].reverse = 0;          // Turning sense
  servo[13].center=90;            // Center Angle
  servo[13].min=70;                // Center Angle
  servo[13].max=110;               // Center Angle
  servo[13].velocidad = 0.25;     // velocity move
  servo[13].pos = servo[8].center;  //Current position


  // *****************************************************************************  
  // Drive Motor 1
  // *****************************************************************************
  motor[0].dirPin = 8;
  motor[0].stepPin = 7;
  motor[0].value = 0;
  motor[0].speedIs = 0;
  motor[0].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 2
  // *****************************************************************************
  motor[1].dirPin = 6;
  motor[1].stepPin = 5;
  motor[1].value = 0;
  motor[1].speedIs = 0;
  motor[1].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 3
  // *****************************************************************************
  motor[2].dirPin = 4;
  motor[2].stepPin = 3;
  motor[2].value = 0;
  motor[2].speedIs = 0;
  motor[2].speedShould = 0;
  // *****************************************************************************  
  // Drive Motor 4
  // *****************************************************************************
  motor[3].dirPin = 2;
  motor[3].stepPin = 1;
  motor[3].value = 0;
  motor[3].speedIs = 0;
  motor[3].speedShould = 0;

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

}

