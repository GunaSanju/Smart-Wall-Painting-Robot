

//motor pins
#define motor1pin1 36
#define motor1pin2 34
#define motor1_a_pin 3
#define motor2pin1 32
#define motor2pin2 30
#define motor2_a_pin 2
#define motor3pin1 40
#define motor3pin2 38
#define motor3_a_pin 4
#define motor4pin1 44
#define motor4pin2 42
#define motor4_a_pin 5

//ultrasonic pin
#define u2_trigpin 28
#define u2_echopin A8

#define u1_trigpin 24
#define u1_echopin A9

//#define u3_trigpin 22
//#define u3_echopin A10


//nema 17
#define stepPin 46 
#define dirPin 48
#define enpin 50

//color sensor pin
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define sensorOut 12

//pump pin
#define pump_pin 53

//stop button
# define startButtonPin  4
# define stopButtonPin  12

// calibrate range
int redMin = 359, redMax = 564;
int greenMin = 363, greenMax = 573;
int blueMin = 320, blueMax = 510;


int refR = -1, refG = -1, refB = -1;

int tolerance = 50;

//PID
double prevError_u1_1 = 0;
double totalError_u1_1 = 0;

double prevError_u2_1 = 0;
double totalError_u2_1 = 0;

double prevError_u1_2 = 0;
double totalError_u1_2 = 0;

double prevError_u2_2 = 0;
double totalError_u2_2 = 0;


double pid_max = 6;

const double desiredstate = (double) 10;
const double kp_1 = 5;
const double kd_1 = 4;
const double ki_1 = 0.01;

const double kp_2 = 6 ;
const double kd_2 = 4;
const double ki_2 = 0.01;

//nema 17
const int stepsPerRevolution = 200;


long totalSteps = 0;
int rotationCount = 0;
bool direction = HIGH; // LOW = down, HIGH = up
bool reversed = false;
bool motionComplete = false;
bool istrue = false;
bool colorsensing = false;
bool painting_start = false;

bool initialComplete = false;
bool emergencyStop = false;

bool is_diff = false;

bool (*print_Matrix)[5];

bool letter_start = false;
bool shape_start = true;
bool rotationComplete = false;


bool paintfilling = false;



//encoder 
const int encoderPin = 21; 
volatile unsigned int pulseCount = 0;
unsigned int pulsesPerRevolution = 40;
unsigned int rotationCount_2 = 0;

volatile unsigned long lastInterruptTime = 0;
const unsigned int debounceDelay = 500;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); 

  //motor pin
   pinMode(motor1pin1,OUTPUT);
  pinMode(motor1pin2,OUTPUT);
  pinMode(motor1_a_pin,OUTPUT);
  pinMode(motor2pin1,OUTPUT);
  pinMode(motor2pin2,OUTPUT);
  pinMode(motor3pin1,OUTPUT);
  pinMode(motor3pin2,OUTPUT);
  pinMode(motor4pin1,OUTPUT);
  pinMode(motor4pin2,OUTPUT);
  pinMode(motor2_a_pin,OUTPUT);
  pinMode(motor3_a_pin,OUTPUT);
  pinMode(motor4_a_pin,OUTPUT);
 
  //ultrasonic pin
  pinMode(u1_trigpin, OUTPUT);
  pinMode(u1_echopin, INPUT);
  pinMode(u2_trigpin, OUTPUT);
  pinMode(u2_echopin, INPUT);
  //pinMode(u3_trigpin, OUTPUT);
  //pinMode(u3_echopin, INPUT);

  //nema 17 pin
   pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enpin, OUTPUT);
  
  digitalWrite(enpin, LOW); 
  digitalWrite(dirPin, direction); 
  

  // color sensor pin
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  //pump pin
  pinMode(pump_pin,OUTPUT);
  digitalWrite(pump_pin,HIGH);

  //stop pin , start pin
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(startButtonPin, INPUT_PULLUP);

  //encoder
  pinMode(encoderPin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

}

//matrix

bool matrix_A[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,1},
    {1,1,1,1,1},
    {1,0,0,0,1},
    {1,0,0,0,1}
};

bool matrix_B[5][5] = {
    {1,1,1,1,0},
    {1,0,0,0,1},
    {1,1,1,1,0},
    {1,0,0,0,1},
    {1,1,1,1,0}
};

bool matrix_C[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,0},
    {1,0,0,0,0},
    {1,0,0,0,0},
    {1,1,1,1,1}
};

bool matrix_D[5][5] = {
    {1,1,1,1,0},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,1,1,1,0}
};

bool matrix_E[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,0},
    {1,1,1,1,0},
    {1,0,0,0,0},
    {1,1,1,1,1}
};
bool matrix_F[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,0},
    {1,1,1,1,0},
    {1,0,0,0,0},
    {1,0,0,0,0}
};

bool matrix_G[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,0},
    {1,0,1,1,1},
    {1,0,0,0,1},
    {1,1,1,1,1}
};

bool matrix_H[5][5] = {
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,1,1,1,1},
    {1,0,0,0,1},
    {1,0,0,0,1}
};

bool matrix_I[5][5] = {
    {1,1,1,1,1},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {1,1,1,1,1}
};

bool matrix_J[5][5] = {
    {1,1,1,1,1},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {1,0,1,0,0},
    {0,1,1,0,0}
};
bool matrix_K[5][5] = {
    {1,0,0,1,0},
    {1,0,1,0,0},
    {1,1,0,0,0},
    {1,0,1,0,0},
    {1,0,0,1,0}
};

bool matrix_L[5][5] = {
    {1,0,0,0,0},
    {1,0,0,0,0},
    {1,0,0,0,0},
    {1,0,0,0,0},
    {1,1,1,1,1}
};

bool matrix_M[5][5] = {
    {1,0,0,0,1},
    {1,1,0,1,1},
    {1,0,1,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1}
};

bool matrix_N[5][5] = {
    {1,0,0,0,1},
    {1,1,0,0,1},
    {1,0,1,0,1},
    {1,0,0,1,1},
    {1,0,0,0,1}
};

bool matrix_O[5][5] = {
    {0,1,1,1,0},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {0,1,1,1,0}
};
bool matrix_P[5][5] = {
    {1,1,1,1,0},
    {1,0,0,0,1},
    {1,1,1,1,0},
    {1,0,0,0,0},
    {1,0,0,0,0}
};

bool matrix_Q[5][5] = {
    {0,1,1,0,0},
    {1,0,0,1,0},
    {1,0,1,1,0},
    {0,1,1,1,0},
    {0,0,0,0,1}
};

bool matrix_R[5][5] = {
    {1,1,1,1,0},
    {1,0,0,0,1},
    {1,1,1,1,0},
    {1,0,1,0,0},
    {1,0,0,1,0}
};

bool matrix_S[5][5] = {
    {0,1,1,1,1},
    {1,0,0,0,0},
    {0,1,1,1,0},
    {0,0,0,0,1},
    {1,1,1,1,0}
};

bool matrix_T[5][5] = {
    {1,1,1,1,1},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {0,0,1,0,0}
};
bool matrix_U[5][5] = {
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {0,1,1,1,0}
};

bool matrix_V[5][5] = {
    {1,0,0,0,1},
    {1,0,0,0,1},
    {0,1,0,1,0},
    {0,1,0,1,0},
    {0,0,1,0,0}
};

bool matrix_W[5][5] = {
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,1,0,1},
    {1,1,0,1,1},
    {1,0,0,0,1}
};

bool matrix_X[5][5] = {
    {1,0,0,0,1},
    {0,1,0,1,0},
    {0,0,1,0,0},
    {0,1,0,1,0},
    {1,0,0,0,1}
};

bool matrix_Y[5][5] = {
    {1,0,0,0,1},
    {0,1,0,1,0},
    {0,0,1,0,0},
    {0,0,1,0,0},
    {0,0,1,0,0}
};

bool matrix_Z[5][5] = {
    {1,1,1,1,1},
    {0,0,0,1,0},
    {0,0,1,0,0},
    {0,1,0,0,0},
    {1,1,1,1,1}
};

bool (*matrix_letterList[26])[5] = {
  matrix_A, matrix_B, matrix_C, matrix_D, matrix_E, matrix_F, matrix_G,
  matrix_H, matrix_I, matrix_J, matrix_K, matrix_L, matrix_M, matrix_N,
  matrix_O, matrix_P, matrix_Q, matrix_R, matrix_S, matrix_T, matrix_U,
  matrix_V, matrix_W, matrix_X, matrix_Y, matrix_Z
};


bool (*matrix_assign_letter(char letter))[5]{
  if (letter >= 'A' && letter <= 'Z') {
      int index = letter - 'A';
      return matrix_letterList[index];
  }
}

bool matrix_ZERO[5][5] = {
    {0,1,1,1,0},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {1,0,0,0,1},
    {0,1,1,1,0}
};

bool matrix_ONE[5][5] = {
    {0,0,1,0,0},
    {0,1,1,0,0},
    {1,0,1,0,0},
    {0,0,1,0,0},
    {1,1,1,1,1}
};

bool matrix_TWO[5][5] = {
    {1,1,1,1,0},
    {0,0,0,0,1},
    {0,0,1,1,0},
    {0,1,0,0,0},
    {1,1,1,1,1}
};

bool matrix_THREE[5][5] = {
    {1,1,1,1,0},
    {0,0,0,0,1},
    {0,0,1,1,0},
    {0,0,0,0,1},
    {1,1,1,1,0}
};

bool matrix_FOUR[5][5] = {
    {1,0,0,1,0},
    {1,0,0,1,0},
    {1,1,1,1,1},
    {0,0,0,1,0},
    {0,0,0,1,0}
};
bool matrix_FIVE[5][5] = {
    {1,1,1,1,1},
    {1,0,0,0,0},
    {1,1,1,1,0},
    {0,0,0,0,1},
    {1,1,1,1,0}
};

bool matrix_SIX[5][5] = {
    {0,1,1,1,0},
    {1,0,0,0,0},
    {1,1,1,1,0},
    {1,0,0,0,1},
    {0,1,1,1,0}
};

bool matrix_SEVEN[5][5] = {
    {1,1,1,1,1},
    {0,0,0,0,1},
    {0,0,0,1,0},
    {0,0,1,0,0},
    {0,1,0,0,0}
};

bool matrix_EIGHT[5][5] = {
    {0,1,1,1,0},
    {1,0,0,0,1},
    {0,1,1,1,0},
    {1,0,0,0,1},
    {0,1,1,1,0}
};

bool matrix_NINE[5][5] = {
    {0,1,1,1,0},
    {1,0,0,0,1},
    {0,1,1,1,1},
    {0,0,0,0,1},
    {0,1,1,1,0}
};

bool (*matrix_digitList[10])[5] = {
  matrix_ZERO, matrix_ONE, matrix_TWO, matrix_THREE, matrix_FOUR,
  matrix_FIVE, matrix_SIX, matrix_SEVEN, matrix_EIGHT, matrix_NINE
};

bool (*matrix_assign_number(char number))[5]{
  if (number >= '0' && number <= '9') {
      int index = number - '1';
      return matrix_digitList[index];
  }
}


bool rectangle[5][5] = {
    {1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1},
    {1, 0, 0, 0, 1},
    {1, 0, 0, 0, 1},
    {1, 1, 1, 1, 1}
    };


bool circle[5][5] = {
    {0, 1, 1, 1, 0},
    {1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1},
    {0, 1, 1, 1, 0}
};

bool triangle[5][5] = {
    {0, 0, 1, 0, 0},
    {0, 1, 1, 1, 0},
    {1, 1, 1, 1, 1},
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};
bool (*matrix_assign_shape(char shape_letter))[5]{
  if (shape_letter == 'R'){
    Serial.println("rect");  
    return rectangle;
  }
  if (shape_letter == 'O'){  
    return circle;
  }
  if (shape_letter == 'T'){  
    return triangle;
  }
}

// measuring distance
double measuring_distance(int trigpin,int echopin){
// ultrasonic distence measuring 
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  float duration = pulseIn(echopin, HIGH);
  
  float distance = duration * 0.034 / 2; 
  return distance;
}


double prevError = 0;
double totalError = 0;

double pid_algo(double currentreading, double& prevError, double& totalError,const double desiredstate, double kp ,double kd , double ki){
  double error = desiredstate - currentreading;

  //pid
  double Pterm = error*kp;
  double Dterm = (error-prevError)*kd;
  double Iterm = totalError*ki;
 
  double pidValue = Pterm + Dterm + Iterm;


  prevError =  error;
  totalError += error;
  
  totalError = constrain(totalError, -10, 10);
  pidValue = constrain(pidValue,-100,100);
  
  return pidValue;


}

//letter and shape painting

bool matrix_read(bool matrix[][5] ,int row ,int column){
  return matrix[row][column];
}
char readCharByIndex(String input, int index) {
  return input.charAt(index);
}

//encoder function 
 //encoder 
int pre_pulseCount = 0 ;
void wheel_rotation_count(){
  pre_pulseCount = pulseCount;
  rotationComplete = false;
  Serial.println(pre_pulseCount);
  while(!rotationComplete){
    rotationCount_2 = (pulseCount-pre_pulseCount) / pulsesPerRevolution;
    // Print rotation count
    Serial.print("Rotations: ");
    Serial.println(rotationCount_2);
    if (rotationCount_2 >= 1){
      rotationComplete = true;
    }

    delay(100);
  }
}

void countPulse() {
  unsigned long interruptTime = micros();
  
  if (interruptTime - lastInterruptTime > debounceDelay) {
    pulseCount++;
    lastInterruptTime = interruptTime;
    }
}
int totalRotations = 50;

void CNC_MoveDown80() {
  totalRotations = 10; 
  rotationCount = 0;
  totalSteps = 0;

  digitalWrite(dirPin, LOW);
  digitalWrite(enpin, LOW);

  while (rotationCount < totalRotations) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);

    totalSteps++;
    if (totalSteps % stepsPerRevolution == 0) {
      rotationCount++;
    }
  }
}

void CNC_MoveUp400() {
  totalRotations = 50; 
  rotationCount = 0;
  totalSteps = 0;

  digitalWrite(dirPin, HIGH); 
  digitalWrite(enpin, LOW);

  while (rotationCount < totalRotations) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    
    totalSteps++;

    if (totalSteps % stepsPerRevolution == 0) {
      rotationCount++;
      Serial.print("Rotation ");
      Serial.print((direction == LOW) ? "down" : "up");
      Serial.print(": ");
      Serial.println(rotationCount);
    }
  }
}

void sprayByMatrixData(bool matrix[][5]){
  for (int col = 0; col < 5; col++) {
      Serial.print("Column: ");
      Serial.println(col);

      // Move down 86 per row
      for (int row = 0; row < 5; row++) {
        Serial.print("Moving to cell: [");
        Serial.print(row);
        Serial.print(", ");
        Serial.print(col);
        Serial.println("]");  
        /*
        double u3_distance = measuring_distance(u3_trigpin,u3_echopin);
        if (u3_distance > 10){
          while(!paintfilling){
            char received = Serial1.read();
            double u3_distance = measuring_distance(u3_trigpin,u3_echopin);
            if (u3_distance < 10 && (received == 'D' || received == 'E' ) ){
              paintfilling = true;
            }
          }
          
          
          if(matrix_read(matrix,row,col) == 1){
            Serial.println("pump ON");
          }
          else{
            Serial.println("pump OFF");
          }
          CNC_MoveDown80();
          Serial.println("pump OFF");
          
        }*/

        if(matrix_read(matrix,row,col) == 1){
              Serial.println("pump ON");
              digitalWrite(pump_pin,LOW);
          }
          else{
            Serial.println("pump OFF");
            digitalWrite(pump_pin,HIGH);
          }
          CNC_MoveDown80();
          Serial.println("pump OFF");
          digitalWrite(pump_pin,HIGH);
      }
      Serial.println("Reversing 400 Cm upward...");
      CNC_MoveUp400();

      // Move forward to next column
      if (col < 4) {
        Serial.println("Shifting to next column...");
        if(abs(abs(u1_pidVal)-abs(u2_pidVal)) <= 6 && abs(abs(u1_pidVal)-abs(u2_pidVal)) >= 0){
          Serial.println("move forward");//move_forward(150);
          move_forward(150);

          wheel_rotation_count();

          Serial.println("move stop");//move_stop();
          move_stop();
        }
        }else if (((u2_pidVal)-(u1_pidVal)) > 7 ){
          Serial.println("u1 side");
          rotate_u1_side(170);
        }else if ((u1_pidVal-(u2_pidVal)) > 7 ){
          Serial.println("u2 side");
          rotate_u2_side(170);
        }
      }
      else{
        Serial.println("Shifting to next letter...");
        Serial.println("move forward_2");//move_forward(150);
        move_forward(150); 

        wheel_rotation_count();

        Serial.println("move stop_2");//move_stop();
        move_stop();
      }
    }
}



/*
void move_right(int speed,double desiredstate,const double kp_2,const double kd_2,const double ki_2) {
  double u1_distance = measuring_distance(u1_trigpin,u1_echopin);
  double u2_distance = measuring_distance(u2_trigpin,u2_echopin);

  double u1_pid = pid_algo(u1_distance, prevError_u1_2, totalError_u1_2, desiredstate,kp_2,kd_2,ki_2);
  double u2_pid = pid_algo(u2_distance, prevError_u2_2, totalError_u2_2, desiredstate,kp_2,kd_2,ki_2);

  double speed_1_2 = 0;
  double speed_3_4 = 0;

  if ( u1_pid > 0.00 && u2_pid > 0.00){
    speed_1_2 = speed - 5*abs(u2_pid);
    speed_3_4 = speed - 5*abs(u1_pid);
  }
  else if ( u1_pid < 0.00 && u2_pid < 0.00){
    speed_1_2 = speed - 5*abs(u1_pid);
    speed_3_4 = speed - 5*abs(u2_pid);
  }
  else if (u1_pid < 0.00 ){
    if(abs(u1_pid) >= abs(u2_pid)){
    speed_1_2 = speed - 5*abs(u1_pid);
    speed_3_4 = speed - 5*abs(u2_pid);
    }
    else{
      speed_1_2 = speed - 5*abs(u2_pid);
    speed_3_4 = speed - 5*abs(u1_pid);
    }
  }
  else if (u2_pid < 0.00 ){
    if(abs(u1_pid) >= abs(u2_pid)){
    speed_1_2 = speed - 5*abs(u2_pid);
    speed_3_4 = speed - 5*abs(u1_pid);
    }
    else{
      speed_1_2 = speed - 5*abs(u1_pid);
    speed_3_4 = speed - 5*abs(u2_pid);
    }
  }
  

  Serial.println(speed_1_2);
  Serial.println(speed_3_4);
  
  Serial.println(u1_pid);
  Serial.println(u2_pid);
  
  digitalWrite(motor1pin1,0);
  digitalWrite(motor1pin2,1);
  analogWrite(motor1_a_pin,speed_1_2);

  digitalWrite(motor2pin1,1);
  digitalWrite(motor2pin2,0);
  analogWrite(motor2_a_pin,speed_1_2);

  digitalWrite(motor3pin1,1);
  digitalWrite(motor3pin2,0);
  analogWrite(motor3_a_pin,speed_3_4 );

  digitalWrite(motor4pin1,0);
  digitalWrite(motor4pin2,1);
  analogWrite(motor4_a_pin,speed_3_4 );
}
*/ 


void move_right(int speed) {
  
  digitalWrite(motor1pin1,0);
  digitalWrite(motor1pin2,1);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,1);
  digitalWrite(motor2pin2,0);
  analogWrite(motor2_a_pin,speed);

  digitalWrite(motor3pin1,1);
  digitalWrite(motor3pin2,0);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,0);
  digitalWrite(motor4pin2,1);
  analogWrite(motor4_a_pin,speed);

}


void move_left(int speed){
  digitalWrite(motor1pin1,1);
  digitalWrite(motor1pin2,0);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,0);
  digitalWrite(motor2pin2,1);
  analogWrite(motor2_a_pin,speed);

  digitalWrite(motor3pin1,0);
  digitalWrite(motor3pin2,1);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,1);
  digitalWrite(motor4pin2,0);
  analogWrite(motor4_a_pin,speed);
}


void move_backward(int speed){
  digitalWrite(motor1pin1,0);
  digitalWrite(motor1pin2,1);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,0);
  digitalWrite(motor2pin2,1);
  analogWrite(motor2_a_pin,speed);

  digitalWrite(motor3pin1,0);
  digitalWrite(motor3pin2,1);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,0);
  digitalWrite(motor4pin2,1);
  analogWrite(motor4_a_pin,speed);
}




void rotate_u2_side( int speed){
  digitalWrite(motor1pin1,1);
  digitalWrite(motor1pin2,0);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,0);
  digitalWrite(motor2pin2,1);
  analogWrite(motor2_a_pin,speed);

  digitalWrite(motor3pin1,0);
  digitalWrite(motor3pin2,0);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,0);
  digitalWrite(motor4pin2,0);
  analogWrite(motor4_a_pin,speed);
}

void rotate_u1_side(int speed){
  digitalWrite(motor1pin1,0);
  digitalWrite(motor1pin2,0);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,0);
  digitalWrite(motor2pin2,0);
  analogWrite(motor2_a_pin,speed);

   digitalWrite(motor3pin1,0);
  digitalWrite(motor3pin2,1);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,1);
  digitalWrite(motor4pin2,0);
  analogWrite(motor4_a_pin,speed);
}

void steppermotor(int distence) {
  // Reset flags at start of function
  motionComplete = false;
  istrue = false;
  reversed = false;
  direction = HIGH;
  rotationCount = 0;
  totalSteps = 0;
  

  digitalWrite(dirPin, direction);
  digitalWrite(enpin, LOW);

  while (!motionComplete && !emergencyStop ) {
    if (Serial1.available()) {
      char received = Serial1.read();
      if (received == 'B') {
        emergencyStop = true;
        digitalWrite(pump_pin,HIGH);
        Serial.println("EMERGENCY STOP ACTIVATED!");
        break;
    }
  }
    

    const int totalRotations = map(distence,0,430,0,51);
    if (rotationCount < totalRotations) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
      digitalWrite(pump_pin,LOW);

      totalSteps++;
      if (totalSteps % stepsPerRevolution == 0) {
        rotationCount++;
        Serial.print("Rotation ");
        Serial.print((direction == LOW) ? "down" : "up");
        Serial.print(": ");
        Serial.println(rotationCount);
      }
    } else {
      if (!reversed) {
        digitalWrite(enpin, HIGH);
        digitalWrite(pump_pin,HIGH);
        delay(1000);
        digitalWrite(pump_pin,HIGH);
        digitalWrite(enpin,LOW);

        direction = !direction;
        digitalWrite(dirPin, direction);

        rotationCount = 0;
        totalSteps = 0;
        reversed = true;

        Serial.println("Reversing direction...");
      } else {
        digitalWrite(enpin, HIGH);
        Serial.println("Motion complete.");
        motionComplete = true;
        istrue = true;
        digitalWrite(pump_pin,HIGH);
        is_diff = false;
      }
    }
  }

  if (istrue) {
    
    
    color_identify();
    move_stop();
  }
}

void color_identify(){
  while (!is_diff){
    int r = map(getRedPW(), redMin, redMax, 255, 0);
    int g = map(getGreenPW(), greenMin, greenMax, 255, 0);
    int b = map(getBluePW(), blueMin, blueMax, 255, 0);

    r = constrain(r, 0, 255);
    g = constrain(g, 0, 255);
    b = constrain(b, 0, 255);

    Serial.print("R: "); Serial.print(r);
    Serial.print(" G: "); Serial.print(g);
    Serial.print(" B: "); Serial.println(b);
    
    move_forward(150);
    Serial.println("forward");

    if (abs(r - refR) < tolerance &&
        abs(g - refG) < tolerance &&
        abs(b - refB) < tolerance) {
      is_diff = true; 
    } else {
      
      
    }

    
  }  
}


// Red
  int getRedPW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    return pulseIn(sensorOut, LOW);
  }

  // Green
  int getGreenPW() {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW);
  }

  // Blue
  int getBluePW() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    return pulseIn(sensorOut, LOW);
  }
void move_forward(int speed){
  digitalWrite(motor1pin1,1);
  digitalWrite(motor1pin2,0);
  analogWrite(motor1_a_pin,speed);

  digitalWrite(motor2pin1,1);
  digitalWrite(motor2pin2,0);
  analogWrite(motor2_a_pin,speed);

  digitalWrite(motor3pin1,1);
  digitalWrite(motor3pin2,0);
  analogWrite(motor3_a_pin,speed);

  digitalWrite(motor4pin1,1);
  digitalWrite(motor4pin2,0);
  analogWrite(motor4_a_pin,speed);


}

void move_stop(){
  digitalWrite(motor1pin1,1);
  digitalWrite(motor1pin2,0);
  analogWrite(motor1_a_pin,0);

  digitalWrite(motor2pin1,0);
  digitalWrite(motor2pin2,1);
  analogWrite(motor2_a_pin,0);

  digitalWrite(motor3pin1,0);
  digitalWrite(motor3pin2,1);
  analogWrite(motor3_a_pin,0);

  digitalWrite(motor4pin1,1);
  digitalWrite(motor4pin2,0);
  analogWrite(motor4_a_pin,0);
}


char received;
char received_1;

void loop() {
  if (Serial1.available()) {
    received = Serial1.read();
    Serial.println(received);
    
    //wall painting
    if (received == 'A' || received == 'D' || received == 'E') {
      painting_start = true;
    }
  } 
  if (painting_start){
    double u1_distance = measuring_distance(u1_trigpin,u1_echopin);
  double u2_distance = measuring_distance(u2_trigpin,u2_echopin);
  Serial.print("u1-> ");
  Serial.print(u1_distance);
  Serial.print(" || ");
  Serial.print("u2-> ");
  Serial.print(u2_distance);
  Serial.print(" || ");
  //get pidValue
  double u1_pidVal = pid_algo(u1_distance, prevError_u1_1, totalError_u1_1, desiredstate,kp_1,kd_1,ki_1);
  double u2_pidVal = pid_algo(u2_distance, prevError_u2_1, totalError_u2_1, desiredstate,kp_1,kd_1,ki_1);
  Serial.print("u1 PID-> ");
  Serial.print(u1_pidVal);
  Serial.print(" / ");
  Serial.print("u2 PID-> ");
  Serial.print(u2_pidVal);
  
  //motor speed controling
  double speed_1 = 175;double speed_2 = 175;
  speed_1 -=(125-abs(u1_pidVal));
  speed_2 -=(125-abs(u2_pidVal));
  delay(10);

  Serial.println("|| "+ String(speed_1));
  Serial.println("|| "+ String(speed_2));
  // movement
  
  if (u1_pidVal < -pid_max && u2_pidVal <-pid_max) {
    if(u1_pidVal <= u2_pidVal){
     move_left(speed_2);
     Serial.println("F 2");
    }else{
      move_left(speed_1);
      Serial.println("F 1");
    }
  } 
    else if (u1_pidVal > pid_max || u2_pidVal > pid_max ) {
      move_right(120);  
      Serial.println("B ");
    }
  else if((u1_pidVal<=pid_max && u1_pidVal>=-pid_max)||(u2_pidVal<=pid_max && u2_pidVal>=-pid_max)) {
     move_stop();
     Serial.println("S");

  if(abs(abs(u1_pidVal)-abs(u2_pidVal)) <= 6 && abs(abs(u1_pidVal)-abs(u2_pidVal)) >= 0){
    Serial.println("stop");

    //letter printing
    if(received == 'D'){
      String text = "";
      bool my_break = false;
      while(true){
        //Serial.println("A");
        while (Serial1.available()) {
          //Serial.println("B");
          char c = Serial1.read();
          Serial.print("Char: ");
          Serial.print(c);
          Serial.print(" | ASCII: ");
          Serial.println((int)c);
          if (c == '\n') {  
            Serial.println("Received: " + text);
            my_break = true;
            break;
            
          } else {
            text += c; 
          }
        }
      if(my_break){
        break;
      } 
      }
      for (int i = 0; i <  text.length(); i++) {
        char letter = readCharByIndex(text, i);
        Serial.println(letter);
        CNC_MoveUp400();
        if ( 65 <= int(letter) && int(letter) <= 90){
          print_Matrix = matrix_assign_letter(letter);
        }
        else{
          print_Matrix = matrix_assign_number(letter);
        }
        
        sprayByMatrixData(print_Matrix);
      }
    }

    // shape painting
    else if(received == 'E'){
      char received_1;
      while(true){
        if (Serial1.available()){
          received_1 = Serial1.read();
          break;
        }
      }
      //CNC_MoveUp400();
      Serial.println(String(received_1));
      print_Matrix = matrix_assign_shape(received_1);
      sprayByMatrixData(print_Matrix);
    }

    //wall painting
    else{
      if (!emergencyStop){
        if (!colorsensing){
            refR = map(getRedPW(), redMin, redMax, 255, 0);
            refG = map(getGreenPW(), greenMin, greenMax, 255, 0);
            refB = map(getBluePW(), blueMin, blueMax, 255, 0);

            refR = constrain(refR, 0, 255);
            refG = constrain(refG, 0, 255);
            refB = constrain(refB, 0, 255);

            Serial.println("Reference color stored:");
            Serial.print("R: "); Serial.print(refR);
            Serial.print(" G: "); Serial.print(refG);
            Serial.print(" B: "); Serial.println(refB);
          
          colorsensing = true;
        }

        
        delay(500);
        steppermotor(430);
        /*
        double u3_distance = measuring_distance(u3_trigpin,u3_echopin);
        Serial.println(u3_distance);
        if (u3_distance < 10){
          steppermotor(430);
        }
        else{
          while(!paintfilling){
            char received = Serial1.read();
            double u3_distance = measuring_distance(u3_trigpin,u3_echopin);
            if (u3_distance < 10 && received == 'A' ){
              paintfilling = true;
            }
            

          }

        }
        */
      }
      else{
        Serial.println("Waiting for START to return to initial position...");
        if (!reversed){
          restart(rotationCount);
        }
        else if (reversed){
          digitalWrite(dirPin,LOW);
          restart(rotationCount);
        }
        
        if (Serial1.available()) {
          char received = Serial1.read();
          if (received == 'C') {
            Serial.print("emergencyStop");
            emergencyStop = false;
          }
        } 
        
      }
    }
      

      /*
      move_right(150,desiredstate,kp_2,kd_2,ki_2);
      delay(29);
      move_stop();
      delay(20);
      move_right(150,desiredstate,kp_2,kd_2,ki_2);
      delay(20);
      Serial.println("move_right");
      */
  if(received == 'D' || received == 'E'){
    while (1); // Stop after one full grid
  }

  }else if (((u2_pidVal)-(u1_pidVal)) > 7 ){
    Serial.println("u1 side");
    rotate_u1_side(170);
  }else if ((u1_pidVal-(u2_pidVal)) > 7 ){
    Serial.println("u2 side");
    rotate_u2_side(170);
  }
  }
}

}
void restart(int rotationCount){
    digitalWrite(dirPin, !direction);
    while (!initialComplete) {
    
      if (rotationCount > 0) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
        digitalWrite(pump_pin,HIGH);

        totalSteps++;
        if (totalSteps % stepsPerRevolution == 0) {
            rotationCount--;
            Serial.print("Rotation ");
            Serial.print((direction == LOW) ? "down" : "up");
            Serial.print(": ");
            Serial.println(rotationCount);
          }
      }
      else {
        initialComplete = true;

      }

    }
}









