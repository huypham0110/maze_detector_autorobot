#include <NewPing.h>

#define TRIGGER_PINL  A0  
#define ECHO_PINL     A1 

#define MAX_DISTANCE 150

#define TRIGGER_PINF  A2  
#define ECHO_PINF     A3
  
#define TRIGGER_PINR  A4  
#define ECHO_PINR     A5  

int dir;

#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

float P = 1 ;
float D = 0.7 ;
float I = 0.01 ;
float oldErrorP ;
float totalError ;
int offset=5 ;

int left_threshold = 6;
int right_threshold = 6 ;
int front_threshold = 4;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;

int en1 = 3 ;
int en2 = 4 ;

int en3 = 13 ;
int en4 = 12 ;

int enA = 5 ;
int enB = 11 ;

int baseSpeed = 65;

long RMS ;
long LMS ;

int led_left = 8 ;
int led_front = 9 ;
int led_right = 10 ;

int tt=0;

NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE);
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

int TestNUM = 2  ;

void setup() {
  for (int i = 2; i <= 13; i++) pinMode(i, OUTPUT);
  first_turn = true ;
  rightWallFollow = true ;
  leftWallFollow = false ;
}

void loop() {
  
  //========================================START========================================//
  ReadSensors();
  if(tt==0){
    if(leftSensor>1 && leftSensor <5) {
      setDirection(FORWARD);
      for(int i=10;i<75;i=i+5){
        analogWrite(enA,i+18);
        analogWrite(enB,i);
        delay(100);
      }
      delay(700);
      tt=1;
    }
  }
  if(tt==1){
    walls();
    if(frontSensor<5){
      setDirection(BACKWARD);
      analogWrite(enA, 60);
      analogWrite(enB, 60);  
    }
    else PID(true) ;
    if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {
      setDirection(STOP);
    }
  }
}

//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW); 
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW); 
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW); 
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH); 
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );
    digitalWrite(en4, LOW );
  }
}

//--------------------------------- Sensors ---------------------------------//
void ReadSensors() {

  leftSensor = sonarLeft.ping_median(1);     
  rightSensor = sonarRight.ping_median(1);   
  frontSensor = sonarFront.ping_median(1);   

  lSensor = sonarLeft.convert_cm(leftSensor);
  rSensor = sonarRight.convert_cm(rightSensor);
  fSensor = sonarFront.convert_cm(frontSensor);

  leftSensor = (lSensor + oldLeftSensor) / 2; 
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;

  oldLeftSensor = leftSensor; 
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;
}

//--------------------------------- control ---------------------------------//

void pid_start() {
  float errorP = leftSensor - rightSensor ;
  float errorD = errorP - oldErrorP;
  float errorI = (2.0 / 3.0) * errorI + errorP ;
  totalError = P * errorP + D * errorD + I * errorI ;
  oldErrorP = errorP ;

  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;

  if (RMS < 0) {
    RMS = map(RMS , 0 , -255, 0, 255);
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(RIGHT);
  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(LEFT);
  }
  else {
    analogWrite(enA , RMS);
    analogWrite(enB , LMS);
    setDirection(FORWARD);
  }
}

//----------------------------- wall follow  control -------------------------------//
void PID( boolean left ) {
  
  if (left == true ) {
    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;
    totalError = P * errorP + D * errorD + I * errorI ;
    oldErrorP = errorP ;
    
    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;
    
    if (RMS < 0) {
      RMS = map(RMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(RIGHT);
    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(LEFT);
    }
    else {
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(FORWARD);
    }
  }
  
  // BAM PHAI ---------------------------------------------
  else {
    float errorP = leftSensor - (rightSensor-2);
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;
    totalError = P * errorP + D * errorD + I * errorI ;
    oldErrorP = errorP ;
    
    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;
    
    if (RMS < 5) {
      RMS=abs(RMS);
      analogWrite(enA , RMS+25);
      analogWrite(enB , LMS);
      setDirection(RIGHT);
    }
    else if (LMS < 5) {
      LMS=abs(LMS);
      analogWrite(enA , RMS+25);
      analogWrite(enB , LMS);
      setDirection(LEFT);
    }
    else {
      analogWrite(enA , RMS);
      analogWrite(enB , LMS);
      setDirection(FORWARD);
    }
  }
}

//--------------------------- wall detection --------------------------------//

void walls() {
  if ( leftSensor < left_threshold ) {
    leftwall = true ;
    digitalWrite(led_left, HIGH);
  }
  else {
    leftwall = false ;
    digitalWrite(led_left, LOW);
  }

  if ( rightSensor < right_threshold ) {
    rightwall = true ;
    digitalWrite(led_right, HIGH);
  }
  else {
    rightwall = false ;
    digitalWrite(led_right, LOW);
  } 
  
  if ( frontSensor < front_threshold ) {
    frontwall = true ;
    digitalWrite(led_front, HIGH);
  }
  else {
    frontwall = false ;
    digitalWrite(led_front, LOW);
  }
}
